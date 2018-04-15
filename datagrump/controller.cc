#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <math.h>

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), current_window( 20 ), ewma_throughput( 6 ),
  prev_wakeup_timestamp( 0 ), bytes_received_since_update( 0 ),
  in_low_throughput_state( false )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << current_window << endl;
  }

  return current_window;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  /* Default: take no action */

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
  if (in_low_throughput_state) {
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt > 60) {
      current_window *= .75;
    } else {
      current_window += .75;
    }
    if (current_window < 4) {
      current_window = 4;
    } 
  }
  bytes_received_since_update += 1424;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

// https://oroboro.com/irregular-ema/
// calculates exponential moving average on irregularly spaced time series data
// (aka acks from the receiver)
double Controller::exponential_moving_average_irregular( 
    double lambda, double sample, double prev_sample, 
    double delta_time, double ema_prev )
{
   double a = delta_time / lambda; 
   double u = exp( a * -1 ); // e^(a*-1)
   double v = ( 1 - u ) / a;
 
   double ema_next = ( u * ema_prev ) + (( v - u ) * prev_sample ) + 
                    (( 1.0 - v ) * sample );
   return ema_next;
}

void Controller::update_current_window(uint time_delta) {
  double sample_throughput = (double)bytes_received_since_update / (double)time_delta;
  double lambda = 0.1;
  double desired_latency = 45; // 200 ms
  ewma_throughput = lambda * sample_throughput + (1 - lambda) * ewma_throughput;

  if (!in_low_throughput_state) {
    current_window = desired_latency * ewma_throughput / 1424;
    // switch states if window is small
    if (current_window < 4) {
      in_low_throughput_state = false;
      current_window = 4;
    }
  } else {
    // switch states if this estimate is higher than current window
    if (desired_latency * ewma_throughput / 1424 > current_window) {
      in_low_throughput_state = false;
    }
  }
  bytes_received_since_update = 0;
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  uint delta = timestamp_ms() - prev_wakeup_timestamp;
  if (delta > 0) {
    update_current_window(delta);
  }
  prev_wakeup_timestamp = timestamp_ms();
  return 1000; /* timeout of one second */
}
