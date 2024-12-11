uint16 magic              #Magic bytes, must be  0xFEAC (hex)    
uint16 version            #version, must be 0x0301 (hex)
uint32 packet_size        #Overall packet size (header+payload)
uint16 header_size        #Header size, defaults to 48 bytes
uint8 distance_ratio      #Distance_ratio, defaults 1
uint8 types               #Type of data, 0x00 : distance only 0x01 : distance and intensity 0x10 : angle and distance
uint16 scan_number        #Sequence for scan (incremented for every scan, starting with 0, overflows)
uint16 packet_number      #Sequence number for packet (counting packets of a particular scan, starting with 1)
uint32 timestamp_decimal  #Raw timestamp of internal clock in NTP time format(decimal part)
uint32 timestamp_integer  #Raw timestamp of internal clock in NTP time format(integer part)
uint16 scan_frequency     #bit[0:14]:Rotating speed(turn/minute),bit[15]:direction
uint16 num_points_scan    #Total number of scan points (samples) within complete scan
uint16 iutput_status      #Iutput status
uint16 output_status      #Output status
uint32 field_status       #Field status
uint16 start_index        #Index of scan start
uint16 end_index          #Index of scan end
uint16 first_index        #Index of first scan point within this packet
uint16 num_points_packet  #Total number of scan points within this packet
uint32 status_flags       #Save