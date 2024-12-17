########################################################################################################################
#
# GNSS ephemeris data definition for one satellite
#
# This message contains information is suitable for RobotGVINS
#
########################################################################################################################

# Timestamp
# NOTE: the header timstamp is used to publish data
#       and will not be used in GNSS data process
Header header

# Satellite PRN
uint32 prn

# Broadcast Ephemeris Data
int32 iode
int32 iodc
float64 sva
int32 svh
int32 week
int32 code
int32 flag
float64 toe
float64 toc
float64 ttr
float64 eph_A
float64 eph_e
float64 i0
float64 OMG0
float64 omg
float64 M0
float64 deln
float64 OMGd
float64 idot
float64 crc
float64 crs
float64 cuc
float64 cus
float64 cic
float64 cis
float64 toes
float64 fit
float64 f0
float64 f1
float64 f2
float64[4] tgd
