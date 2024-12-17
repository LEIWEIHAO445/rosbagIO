int32         VERSION = 2

string        sv_id

string        sig_id

int8          sig_band
int8          SIG_BAND_L1 = 1
int8          SIG_BAND_L2 = 2
int8          SIG_BAND_L5 = 5

float32       sig_prres

float32       sig_cno

int8          sig_glo_fcn

bool          sig_healthy

bool          sig_pr_used
bool          sig_cr_used
bool          sig_do_used

bool          sig_pr_corr
bool          sig_cr_corr
bool          sig_do_corr

int8          sig_use
int8          SIG_USE_NONE         = 0
int8          SIG_USE_SEARCHING    = 1
int8          SIG_USE_ACQUIRED     = 2
int8          SIG_USE_UNUSABLE     = 3
int8          SIG_USE_CODE_LOCKED  = 4
int8          SIG_USE_CARR_LOCKED  = 5

int8          sig_corr
int8          SIG_CORR_NONE        = 0
int8          SIG_CORR_OSR         = 1
int8          SIG_CORR_SSR         = 2
int8          SIG_CORR_OTHER       = 3

int32         ix_sat
int32         ix_obs