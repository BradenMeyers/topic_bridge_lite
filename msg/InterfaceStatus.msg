# InterfaceStatus.msg

# Queue status constants
uint8 DISCONNECTED=0
uint8 OK=1
uint8 CONGESTED=2
uint8 OVERLOADED=3

std_msgs/Header header
uint8  queue_depth
uint8  queue_max
uint8  dropped_last_window   # frames dropped because the send queue was full
uint8  seq_gaps_last_window  # missing frames from encode node detected via sequence number gaps 
uint8  medium_state
uint32 bytes_sent_last_window  # raw byte count over the last window
uint16 window_ms               # duration of the accounting window in ms