int32         VERSION = 1

string        sv_id

string        sig_id

int8          sig_band
int8          SIG_BAND_L1 = 1
int8          SIG_BAND_L2 = 2
int8          SIG_BAND_L5 = 5

float64       pr_meas
float64       cp_meas
float64       do_meas
float32       pr_std
float32       cp_std
float32       do_std
bool          pr_valid
bool          cp_valid
bool          do_valid

int8          sig_glo_fcn
float32       sig_cno
float32       locktime

bool          half_cyc_valid
bool          half_cyc_sub

int32         ix_sat
int32         ix_sig