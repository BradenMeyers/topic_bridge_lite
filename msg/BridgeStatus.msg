std_msgs/Header header
uint16 window_ms

float32 estimated_throughput_bps  # EWMA channel capacity estimate
float32 output_bps                # compressed bits/sec actually transmitted over wire
float32 input_bps                 # uncompressed bits/sec received from ROS topics

uint32 packets_in_last_window      # frames encoder submitted to socket
uint32 packets_out_last_window     # frames actually transmitted over the wire
uint32 packets_skipped_last_window # frames skipped by encoder (budget exceeded)
uint32 packets_dropped_last_window # frames dropped at socket queue (queue full)
float32 packet_loss_rate           # (skipped + dropped) / (in + skipped) [0.0-1.0]

uint8 queue_depth
uint8 queue_max
uint8 medium_state
