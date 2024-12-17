########################################################################################################################
#
# GNSS observation data definition for all satellites in one epoch
#
# This message contains information is suitable for RobotGVINS
#
########################################################################################################################

# Timestamp
Header header

# Observation Info
uint32     flag
uint32     nsat
uint32[]   ngnss

# Observation data for all satellites
RobotGVINS_GNSSSat[] obsdata