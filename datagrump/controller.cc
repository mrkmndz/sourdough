#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <math.h>

#define PKT_SIZE 1
#define MIN_WINDOW_SIZE 4
#define BASELINE_RTT 100
#define BASELINE_BW 1
#define RTT_TIMEOUT 300000
#define BW_TIMEOUT 600
#define PG_FREQ 100
using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_(false || debug ), bytes_delivered(0), last_arrival(timestamp_ms()),
  pm_mutex(), packet_map(), nextSendTime(timestamp_ms()), rw_mutex(), rtt_window(), bw_mutex(), bw_window(),
  cached_rtt(BASELINE_RTT), cached_bw(BASELINE_BW), pacing_gain_index(0)
{}

double Controller::window_scan(std::deque<window_entry>& window, double baseline, bool max, uint64_t timeout) {
  if (window.empty()) {
    return baseline;
  }
  uint64_t now = timestamp_ms();
  while (!window.empty() && window.back().time > timeout && window.back().time < now - timeout) {
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

void Controller::update_min_rtt(double rtt) {
  std::lock_guard<std::mutex> guard(rw_mutex);
  window_entry entry;
  entry.value = rtt;
  entry.time = timestamp_ms();
  rtt_window.push_front(entry);
  cached_rtt = window_scan(rtt_window, BASELINE_RTT, false, RTT_TIMEOUT);
}

void Controller::update_max_bw(double bw){
  std::lock_guard<std::mutex> guard(bw_mutex);
  window_entry entry;
  entry.value = bw;
  entry.time = timestamp_ms();
  bw_window.push_front(entry);
  cached_bw = window_scan(bw_window, BASELINE_BW, true, BW_TIMEOUT);
}

void Controller::cycle_pacing_gain(){
  static uint64_t last_update = 0;
  uint64_t now = timestamp_ms();
  if (now - last_update > PG_FREQ) {
    last_update = now;
    pacing_gain_index = (pacing_gain_index + 1) % 8;
  }
}

/* Get current window size, in datagrams */
bool Controller::should_send(uint64_t inflight)
{
  cycle_pacing_gain();

  auto bdp = cached_rtt * cached_bw;
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " bdp is " << bdp << endl;
  }
  if (bdp < 10) {
    bdp = 10;
  }

  return inflight < bdp && timestamp_ms() >= nextSendTime;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
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

  auto pacing_gain = pacing_gains[pacing_gain_index];
  nextSendTime = timestamp_ms() + PKT_SIZE / ( pacing_gain * cached_bw );

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
  }
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
  update_min_rtt(rtt);

  bytes_delivered += PKT_SIZE;
  last_arrival = timestamp_ms();

  std::lock_guard<std::mutex> guard(pm_mutex);
  packet_state state = packet_map[sequence_number_acked];
  packet_map.erase(sequence_number_acked);

  float delivery_rate = ((float) (bytes_delivered - state.bytes_delivered_before_sending)) / (timestamp_ms() - state.last_arrival_before_sending);

  update_max_bw(delivery_rate);

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
