Header header
int32 ts        # sec
# Mode is 1 byte size:
# Bit #:      | Set to 0:                             | Set to 1:
# ----------- | ------------------------------------- | -------------------------------------
# bit 0       | Incremental control,                  | Absolute control,
#             | the angle reference is the            | the angle reference is
#             | current Gimbal location               | related to configuration in DJI Go App
# bit 1       | Gimbal will follow the command in Yaw | Gimbal will maintain position in Yaw
# bit 2       | Roll invalid bit, the same as bit[1]  | Roll invalid bit, the same as bit[1]
# bit 3       |Pitch invalid bit, the same as bit[1]  | Pitch invalid bit, the same as bit[1]
# bit [4:7]   | bit [4:7]: reserved, set to be 0      |
uint8 mode
float32 pitch   # rads
float32 yaw     # rads
float32 roll    # rads
