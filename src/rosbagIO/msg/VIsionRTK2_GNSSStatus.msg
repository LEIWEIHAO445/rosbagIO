int32         VERSION = 2
Header        header
bool          msg_valid                # Indicates if the message fields contain valid data
int16         time_gps_wno             # GPS week number [-]
float64       time_gps_tow             # GPS time of week [s]
bool          time_gps_ok              # time_gps_wno and time_gps_tow contain useful time
int16         utc_year                 # UTC year
int8          utc_month                # UTC month (1 = January, ..., 12 = December)
int8          utc_day                  # UTC day of month (1 ... 31)
int8          utc_hour                 # UTC hour of day (0 ... 23)
int8          utc_minute               # UTC minute of hour (0 ... 59)
float32       utc_second               # UTC second of minute (0 ... 60)
bool          utc_ok                   # UTC time fully known, confirmed and unambiguous
float64       pos_lat                  # Latitude [deg]
float64       pos_lon                  # Longitude [deg]
float32       pos_height               # Ellipsoidal height [m]
float32       pos_acc_h                # Horizontal position accuracy estimate [m]
float32       pos_acc_v                # Vertical position accuracy estimate [m]
bool          pos_valid                # pos_* fields contain valid data. See also the fix_* fields below!
bool          fix_ok                   # Fix is OK (not masked or otherwise deemed fishy)
int8          fix_type                 # GNSS fix type, one of:
int8          FIX_TYPE_UNKNOWN       = 0 # Unknown fix
int8          FIX_TYPE_NOFIX         = 1 # No fix
int8          FIX_TYPE_DRONLY        = 2 # Dead-reckoning only fix
int8          FIX_TYPE_TIME          = 3 # Time only fix (not used on F9P in rover mode)
int8          FIX_TYPE_S2D           = 4 # (Single) 2D fix
int8          FIX_TYPE_S3D           = 5 # (Single) 3D fix
int8          FIX_TYPE_S3D_DR        = 6 # (Single) 3D + dead-reckoning fix (not used on F9P)
int8          FIX_TYPE_RTK_FLOAT     = 7 # RTK float fix (implies S3D fix)
int8          FIX_TYPE_RTK_FIXED     = 8 # RTK fixed fix (implies S3D fix)
int8          FIX_TYPE_RTK_FLOAT_DR =  9 # RTK float fix + dead-reckoning (implies S3D_DR fix)
int8          FIX_TYPE_RTK_FIXED_DR = 10 # RTK fixed fix + dead-reckoning (implies S3D_DR fix)
int8[12]      sig_cno_hist_trk         # Number of signals per signal level for signals tracked
int8[12]      sig_cno_hist_nav         # Number of signals per signal level for signals used in navigation
int8          sig_cno_num_trk          # Total number of signals tracked
int8          sig_cno_num_nav          # Total number of signals used in navigation
int8          sol_num_sat              # Number of satellites used in solution
int8          sol_num_sat_gps          # Number of GPS satellites used in solution
int8          sol_num_sat_glo          # Number of GLONASS satellites used in solution
int8          sol_num_sat_gal          # Number of Galileo satellites used in solution
int8          sol_num_sat_bds          # Number of BeiDou satellites used in solution
int8          sol_num_sat_other        # Number of other (QZSS, SBAS, ...) satellites used in solution
int8          sol_num_sig              # Number of signals used in solution
int8          sol_num_sig_gps          # Number of GPS signals used in solution
int8          sol_num_sig_glo          # Number of GLONASS signals used in solution
int8          sol_num_sig_gal          # Number of Galileo signals used in solution
int8          sol_num_sig_bds          # Number of BeiDou signals used in solution
int8          sol_num_sig_other        # Number of other (QZSS, SBAS, ...) signals used in solution
int8          sol_num_sig_corr_L1      # Number of used signals with OSR or SSR correction data L1 band
int8          sol_num_sig_corr_L2      # Number of used signals with OSR or SSR correction data L2 band
float32       sol_pdop                 # Position DOP
int8          sat_num_vis              # Number of satellites theoretically visible (above the horizon)
int8          sat_num_vis_gps          # Number of GPS satellites theoretically visible (above the horizon)
int8          sat_num_vis_glo          # Number of GLONASS satellites theoretically visible (above the horizon)
int8          sat_num_vis_gal          # Number of Galileo satellites theoretically visible (above the horizon)
int8          sat_num_vis_bds          # Number of BeiDou satellites theoretically visible (above the horizon)
int8          sat_num_vis_other        # Number of other (QZSS, SBAS, ...) satellites theoretically visible (above the horizon)
bool          rf_valid                 # rf_* fields valid flag
int32[2]      rf_noise                 # RF noise level (block 1 and 2) [%]
float32[2]    rf_agc                   # RF automatic gain control level (block 1 and 2) [%]
float32[2]    rf_jam                   # RF CW jamming indicator (block 1 and 2) [-]
float32       rel_pos_n                # Relative position vector North component [m]
float32       rel_pos_e                # Relative position vector East component [m]
float32       rel_pos_d                # Relative position vector Down component [m]
float32       rel_acc_n                # Relative position vector North component accuracy estimate [m]
float32       rel_acc_e                # Relative position vector East component accuracy estimate [m]
float32       rel_acc_d                # Relative position vector Down component accuracy estimate [m]
bool          rel_valid                # rel_* fields valid
bool          ant_valid                # ant_* fields valid flag
int8          ant_state                # Antenna supervisor state
int8          ANT_STATE_UNKNOWN  = 0   # State is unknown
int8          ANT_STATE_INIT     = 1   # Antenna supervisor is initialising
int8          ANT_STATE_OK       = 2   # Antenna detected and okay
int8          ANT_STATE_OPEN     = 3   # No antenna detected
int8          ANT_STATE_SHORT    = 4   # Antenna short circuit detected
int8          ant_power                # Antenna power state
int8          ANT_POWER_UNKNOWN  = 0   # Antenna power supply unknown
int8          ANT_POWER_OFF      = 1   # Antenna power supply is off
int8          ANT_POWER_ON       = 2   # Antenna power supply is on