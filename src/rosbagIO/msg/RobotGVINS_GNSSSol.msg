########################################################################################################################
#
# GNSS solution message definition
#
# This message contains information is suitable for RobotGVINS
#
########################################################################################################################

# Sequence, timestamp, frame ID
# - Note that the header.stamp is defined as GPSTime
Header        header

# Solution status
uint8        Q         # Qfactor
uint8        AmbFix    # Ambiguity Fix Type
float32      DDOP      # DDOP

# position and velocity in ECEF frame
float64[3]       pos_XYZ
float64[3]       vel_XYZ

# position and velocity covariance
float64[9]       cov_pos_XYZ
float64[9]       cov_vel_XYZ
