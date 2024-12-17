int32         VERSION = 2

string        sv_id

int8          sv_orb
int8          SV_ORB_NONE  = 0
int8          SV_ORB_EPH   = 1
int8          SV_ORB_ALM   = 2
int8          SV_ORB_PRED  = 3
int8          SV_ORB_OTHER = 4

int8          sv_elev
int16         sv_azim

int32[]       ix_sig
int32[]       ix_obs