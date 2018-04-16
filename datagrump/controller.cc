#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <math.h>

#define PKT_SIZE 1424
#define MIN_WINDOW_SIZE 4
using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
{}

/* Get current window size, in datagrams */
bool Controller::should_send(uint64_t inflight)
{

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << current_window << endl;
  }

  auto bdp = min_rtt() * max_bw();
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
  packet_map[sequence_number] = state;

  nextSendTime = timestamp_ms() + SIZE / ( pacing_gain * max_bw() )

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

  bytes_delivered += SIZE;
  last_arrival = timestamp_ms();

  packet_state state = packet_map[sequence_number_acked];
  packet_map.erase(sequence_number_acked);

  auto delivery_rate = (bytes_delivered - state.bytes_delivered_before_sending) / (timestamp_ms() - state.last_arrival_before_sending);

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
