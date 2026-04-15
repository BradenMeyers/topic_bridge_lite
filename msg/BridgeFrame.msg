# Internal envelope between encoder_node and transport_node.
# The encoder builds the complete wire packet and stores it in payload.
# All other fields mirror bytes already inside payload — for debug/monitoring only.
# Transport nodes simply call write(msg.payload) with no further manipulation.
#
# Wire layout without addressing: [topic_id][sequence][zstd(ros2_bytes)...]
# Wire layout with addressing:    [dst_addr][src_addr][topic_id][sequence][zstd(ros2_bytes)...]
uint8 dst_addr    # destination address mirror (0x00 = unset/broadcast)
uint8 src_addr    # source address mirror (0x00 = unset)
uint8 topic_id    # registry ID mirror
uint8 sequence    # rolling counter mirror
uint8[] payload   # complete wire-ready packet
