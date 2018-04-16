#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>


/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */

  /* Add member variables here */
  struct packet_state_t {
    uint64_t bytes_delivered_before_sending;
    uint64_t last_arrival_before_sending;
  };
  typedef struct packet_state_t packet_state;

  uint64_t bytes_delivered;
  uint64_t last_arrival;
  std::mutex pm_mutex;
  std::map<uint64_t, packet_state> packet_map;

  uint64_t nextSendTime;

  struct window_entry_t {
    double value;
    uint64_t time;
  };
  typedef struct window_entry_t window_entry;
  uint64_t window_scan(std::deque<window_entry>& window, double baseline, bool max, uint64_t timeout);

  std::mutex rw_mutex;
  std::deque<window_entry> rtt_window;
  std::mutex bw_mutex;
  std::deque<window_entry> bw_window;
  double min_rtt();
  double max_bw();
  void update_min_rtt(double rtt);
  void update_max_bw(double bw);

  int pacing_gain_index;
  const double pacing_gains[8] = {5/4, 3/4, 1, 1, 1, 1, 1, 1};
  void cycle_pacing_gain();


public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  bool should_send(uint64_t inflight);

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp,
			  const bool after_timeout );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms();
};

#endif
