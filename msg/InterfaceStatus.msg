# InterfaceStatus.msg

# Queue status constants
uint8 DISCONNECTED=0
uint8 OK=1
uint8 CONGESTED=2
uint8 OVERLOADED=3

std_msgs/Header header
uint8  queue_depth
uint8  queue_max
uint8  dropped_last_window
uint8  medium_state
uint32 bytes_sent_last_window  # raw count, no interpretation
uint16 window_ms               # so encoder knows what window produced that count