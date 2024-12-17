########################################################################################################################
#
# GNSS observation data definition for one satellite
#
# This message contains information is suitable for RobotGVINS
#
########################################################################################################################

# Satellite PRN
uint32 prn

# Observation Data
float64[] cp_meas   # observation data carrier-phase (cycle)
float64[] pr_meas   # observation data pseudorange (m)
float64[] do_meas   # observation data doppler frequency (Hz)
float32[] sig_cno   # signal strength
string[] code       # P code type (channel)
uint8[] SNR         # signal strength (0.25 dBHz)
uint8[] LLI         # loss of lock indicator
float64[] cs        # simulated cycle slip
float64[] P_TGD     # observation data pseudorange after TGD correction(m)
float64[] SMP       # carrier mmoothing of code pseudoranges