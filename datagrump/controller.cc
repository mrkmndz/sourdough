#include <iostream>
#include <ctime>
#include <string>

#include "controller.hh"
#include "timestamp.hh"
#include "util.hh"
#include <math.h>

#define PKT_SIZE 1400
#define MIN_WINDOW_SIZE 4
#define BASELINE_RTT 100
#define BASELINE_BW 100
#define RTT_TIMEOUT 100000
using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_(false || debug ), bytes_delivered(0), last_arrival(timestamp_ms()),
  pm_mutex(), packet_map(), nextSendTimeNs(0), rw_mutex(), rtt_window(), bw_mutex(), bw_window(),
  cached_rtt(BASELINE_RTT), cached_bw(BASELINE_BW), pacing_gain(2.0)
{}

double Controller::window_scan(std::deque<window_entry>& window, double baseline, bool max, uint64_t timeout) {
  if (window.empty()) {
    return baseline;
  }
  uint64_t now = timestamp_ms();
  while (!window.empty() && now > timeout && window.back().time < now - timeout) {
    window.pop_back();
  }
  if (window.empty()) {
    return baseline;
  }
  double selected = max ? 0 : UINT64_MAX;
  for (auto entry : window) {
    if (
        (max && entry.value > selected) || 
        (!max && entry.value < selected) ){
      selected = entry.value;
    }
  }
  return selected;
}

void Controller::update_min_rtt(double rtt, uint64_t send_time) {
  std::lock_guard<std::mutex> guard(rw_mutex);
  window_entry entry;
  entry.value = rtt;
  entry.time = send_time;
  rtt_window.push_front(entry);
  cached_rtt = window_scan(rtt_window, BASELINE_RTT, false, RTT_TIMEOUT);
}

void Controller::update_max_bw(double bw, uint64_t send_time){
  std::lock_guard<std::mutex> guard(bw_mutex);
  window_entry entry;
  entry.value = bw;
  entry.time = send_time;
  entry.time = timestamp_ms();
  bw_window.push_front(entry);
  cached_bw = window_scan(bw_window, BASELINE_BW, true, 2.5*cached_rtt);
}

void Controller::cycle_pacing_gain(){
  static uint64_t last_update = 0;
  static int pacing_gain_index = 0;
  static const double pacing_gains[8] = {1, 1, 1, 1.25, .75, 1, 1, 1};
  uint64_t now = timestamp_ms();
  if (now - last_update > cached_rtt/2) {
    last_update = now;
    pacing_gain_index = (pacing_gain_index + 1) % 8;
    pacing_gain = pacing_gains[pacing_gain_index];
  }
}

/* nanoseconds per millisecond */
static const uint64_t MILLION = 1000000;

/* nanoseconds per second */
static const uint64_t BILLION = 1000 * MILLION;

static timespec current_time()
{
  timespec ret;
  SystemCall( "clock_gettime", clock_gettime( CLOCK_REALTIME, &ret ) );
  return ret;
}

static uint64_t ns_raw( const timespec & ts )
{
    const uint64_t nanos = ts.tv_sec * BILLION + ts.tv_nsec;
    return nanos;
}


uint64_t now_ns( const timespec & ts )
{
    const static uint64_t EPOCH = ns_raw( current_time() );
    return ns_raw( ts ) - EPOCH;
}

uint64_t now_ns() {
  return now_ns(current_time());
}

/* Get current window size, in datagrams */
bool Controller::should_send(uint64_t inflight)
{

  auto bdp = cached_rtt * cached_bw;
  cycle_pacing_gain();
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " bdp is " << bdp << endl;
  }
  if (bdp < 1) {
    bdp = 1;
  }

  auto limit = bdp * 1.2;

  bool full = inflight * PKT_SIZE > limit;
  bool waiting = now_ns() < nextSendTimeNs;
  /*
  static std::string str = "";
  str = str + "at " + std::to_string(timestamp_ms());
  str = str + "bw: " + std::to_string(cached_bw);
  str = str + ", rtt: " + std::to_string(cached_rtt);
  str = str + ", limit: " + std::to_string(bdp);
  if (full) {
    str = str + "full\n";
  } else if (waiting) {
    str = str + "waiting\n";
  } else {
    str = str + "approved!\n";
  }
  static int count = 0;
  count++;
  if (count % 100 == 0) {
    cout << str << endl;
    str = "";
  }
  */

  return !full;
  return !full && !waiting;
}

/* A datagram was sent */
uint64_t Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  packet_state state;
  state.bytes_delivered_before_sending = bytes_delivered;
  state.last_arrival_before_sending = last_arrival;
  std::lock_guard<std::mutex> guard(pm_mutex);
  packet_map[sequence_number] = state;

  uint64_t intervalNs = ((double) PKT_SIZE * MILLION) / ( cached_bw * pacing_gain );
  if (intervalNs > MILLION) {
    intervalNs = MILLION;
  }

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
  }
  return intervalNs;
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
  update_min_rtt(rtt, send_timestamp_acked);

  bytes_delivered += PKT_SIZE;
  last_arrival = timestamp_ms();

  std::lock_guard<std::mutex> guard(pm_mutex);
  packet_state state = packet_map[sequence_number_acked];
  packet_map.erase(sequence_number_acked);

  float delivery_rate = ((float) (bytes_delivered - state.bytes_delivered_before_sending)) / (timestamp_ms() - state.last_arrival_before_sending);

  update_max_bw(delivery_rate, send_timestamp_acked);

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 1000; /* timeout of one second */
}
