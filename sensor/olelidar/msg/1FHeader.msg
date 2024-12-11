uint32 magic              #Magic bytes, must be  0xFEF0010F (hex)    
uint16 version            #version, must be 0x0200 (hex)
uint8  distance_ratio     #Distance scale, distance = readout data x distance scale
uint8[3]  brand           #Brand name code, use capital letters and digits. Using “\0” for missing code
uint8[12]  vommercial     #Commercial type code: ended with “\0”
uint16 internal           #Internal type code
uint16 hardware           #Hardware version
uint16 software           #Software version
uint32 timestamp          #When NTP is OFF: Unit ms. represents the number of millisecondsafter power-on
                          #When NTP is ON: Represents the fractional part of a timestamp in NTP64 format.
uint16 scan_frequency     #bit[0:14]:Rotating speed(turn/minute),bit[15]:direction
uint8 safe_status         #Safe zone status，same as the hardware INPUT/OUTPUT
uint8 error_status        #Error status
uint32 status_flags       #Save