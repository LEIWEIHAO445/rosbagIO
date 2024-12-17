int32         VERSION = 3

Header        header
time          sys_stamp

int16         time_gps_wno
float64       time_gps_tow
bool          time_gps_ok

bool          fix_ok
int8          fix_type
int8          FIX_TYPE_UNKNOWN      =  0  # keep in sync with GnssStatus!
int8          FIX_TYPE_NOFIX        =  1
int8          FIX_TYPE_DRONLY       =  2
int8          FIX_TYPE_TIME         =  3
int8          FIX_TYPE_S2D          =  4
int8          FIX_TYPE_S3D          =  5
int8          FIX_TYPE_S3D_DR       =  6
int8          FIX_TYPE_RTK_FLOAT    =  7
int8          FIX_TYPE_RTK_FIXED    =  8
int8          FIX_TYPE_RTK_FLOAT_DR =  9
int8          FIX_TYPE_RTK_FIXED_DR = 10

float64       pos_X
float64       pos_Y
float64       pos_Z

float64       vel_X
float64       vel_Y
float64       vel_Z

float64[9]    cov_pos_xyz
float64[9]    cov_vel_xyz

int8          sol_num_sat
float32       sol_pdop

float32       acc_pos_h
float32       acc_pos_v
float32       acc_vel

bool          clk_valid
float64       clk_dt
float64       clk_ddt
float64       clk_dt_acc
float64       clk_ddt_acc

VisionRTK2_GNSSSat[] sat
VisionRTK2_GNSSSig[] sig

bool          att_valid
float64       att_roll
float64       att_pitch
float64       att_heading
float32       att_acc_roll
float32       att_acc_pitch
float32       att_acc_heading
geometry_msgs/Quaternion att_quat

int16         obs_gps_wno
float64       obs_gps_tow
VisionRTK2_GNSSObs[] obs