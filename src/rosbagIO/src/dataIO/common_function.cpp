/**
 * @brief
 */

#include "rosbagIO_header.h"

/**
 * @brief       Copy the string from the src to the dst
 * @note
 *
 * @param[in]   src          char     source string
 * @param[in]   nPos         int      the start location
 * @param[in]   nCount       int      the number of char
 * @param[out]  dst          char     destination string
 *
 * @return      void
 */
void xstrmid(const char *src, const int nPos, const int nCount, char *dst)
{
    int i;
    const char *str;
    char c;

    str = src + nPos;

    for (i = 0; i < nCount; i++)
    {
        c = *(str + i);
        if (c)
        {
            *(dst + i) = c;
        }
        else
        {
            // elimate the '\n' in the end
            if (dst[i - 1] == '\n')
                dst[i - 1] = '\0';
            *(dst + i) = '\0';
            break;
        }
    }

    *(dst + nCount) = '\0';
}

namespace gnss_common
{
    /**
     * @brief       Convert the second to GPSWeek and GPSSecond
     * @note
     *
     * @param[in]   double      sec      seconds
     * @param[out]
     *
     * @return      IPS_GPSTIME
     */
    IPS_GPSTIME toGPSTIME(double sec)
    {
        return IPS_GPSTIME(int(sec / 604800.0), fmod(sec, 604800.0));
    }

    /**
     * @brief       Convert the customized PRN to GNSS PRN
     * @note        The start index of BDS3 is 1
     *
     * @param[in]   prn         int      customized PRN
     * @param[out]  sys         int      Navigation System [SYSGPS SYSGLO SYSBD2 SYSBD3 SYSGAL SYSQZS]
     *
     * @return      int         GNSS PRN
     */
    int satprn2no(const int prn, int *sys)
    {
        int sat = prn;
        int SYS = IPS_SYSNON;

        if (prn <= 0 || IPS_NSATMAX < prn)
        {
            sat = 0;
        }
        else if ((prn - IPS_PRNGPS) <= IPS_NSATGPS)
        {
            SYS = IPS_SYSGPS;
            sat -= IPS_PRNGPS;
        }
        else if ((prn - IPS_PRNGLO) <= IPS_NSATGLO)
        {
            SYS = IPS_SYSGLO;
            sat -= IPS_PRNGLO;
        }
        else if ((prn - IPS_PRNBD2) <= IPS_NSATBD2)
        {
            SYS = IPS_SYSBD2;
            sat -= IPS_PRNBD2;
        }
        else if ((prn - IPS_PRNBD3) <= IPS_NSATBD3)
        {
            SYS = IPS_SYSBD3;
            sat -= IPS_PRNBD3;
        }
        else if ((prn - IPS_PRNGAL) <= IPS_NSATGAL)
        {
            SYS = IPS_SYSGAL;
            sat -= IPS_PRNGAL;
        }
        else if ((prn - IPS_PRNQZS) <= IPS_NSATQZS)
        {
            SYS = IPS_SYSQZS;
            sat -= IPS_PRNQZS;
        }
        else
        {
            sat = 0;
        }

        if (sys)
            *sys = SYS;

        return sat;
    }

    /**
     * @brief       Convert the customized PRN to GNSS PRN (Gxx)
     * @note
     *
     * @param[in]   prn         int      the customized PRN in the IPS program
     *
     * @return      string      Gnn Rnn Cnn Enn Jnn (error: Nnn)
     */
    std::string satprn2no(const int prn)
    {
        char str[5] = {'\0'};
        int sat, sys;
        sat = satprn2no(prn, &sys);
        char s;

        switch (sys)
        {
        case IPS_SYSGPS:
            s = 'G';
            break;
        case IPS_SYSGLO:
            s = 'R';
            break;
        case IPS_SYSBD2:
            s = 'C';
            break;
        case IPS_SYSBD3:
            s = 'C';
            break;
        case IPS_SYSGAL:
            s = 'E';
            break;
        case IPS_SYSQZS:
            s = 'J';
            break;
        default:
            s = 'N';
        }

        // The start location of BDS3
        if (sys == IPS_SYSBD3)
            sat += IPS_NSATBD2;

        sprintf(str, "%c%02d", s, sat);

        return std::string(str);
    }

    /**
     * @brief       Convert the GNSS PRN to the customized PRN
     * @note        The start index of BDS3 is 1
     *
     * @param[in]   no      char     GNSS PRN (Gno Rno Cno Eno Jno)
     * @param[out]
     *
     * @return      int     customized PRN
     */
    int satno2prn(const char *no)
    {
        int prn = 0;
        char code;

        if (sscanf(no, "%d", &prn) == 1)
        {
            if (1 <= prn && prn <= IPS_NSATGPS)
                prn += IPS_PRNGPS;
            else
                prn = 0;
            return prn;
        }

        if (sscanf(no, "%c%d", &code, &prn) < 2)
            return 0;

        switch (code)
        {
        case 'G':
            if (prn > IPS_NSATGPS || prn < 1)
                return 0;
            prn += IPS_PRNGPS;
            break;
        case 'R':
            if (prn > IPS_NSATGLO || prn < 1)
                return 0;
            prn += IPS_PRNGLO;
            break;
        case 'E':
            if (prn > IPS_NSATGAL || prn < 1)
                return 0;
            prn += IPS_PRNGAL;
            break;
        case 'J':
            if (prn > IPS_NSATQZS || prn < 1)
                return 0;
            prn += IPS_PRNQZS;
            break;
        case 'C':
        {
            // BD3 test satellites will be neglected
            if (prn == 31 || prn == 56 || prn == 57 || prn == 58)
                return 0;

            if (prn > 0 && prn <= IPS_NSATBD2)
                prn += IPS_PRNBD2;
            else if (prn > IPS_NSATBD2 && prn <= IPS_NSATBD2 + IPS_NSATBD3)
                prn += IPS_PRNBD3 - IPS_NSATBD2;
            else
                return 0;

            break;
        }

        default:
            return 0;
        }

        return prn;
    }

    /**
     * @brief       Find the index based on the system
     * @note        This can also be applied for BDS2 and BDS3
     *
     * @param[in]   sys       int      Navigation System [SYSGPS SYSGLO SYSBD2 SYSBD3 SYSGAL SYSQZS]
     * @param[out]
     *
     * @return      int       the index
     *
     */
    int Sys2Index(int sys)
    {
        switch (sys)
        {
        case IPS_SYSGPS:
            return IPS_ISYSGPS;
        case IPS_SYSGLO:
            return IPS_ISYSGLO;
        case IPS_SYSBD2:
            return IPS_ISYSBD2;
        case IPS_SYSBD3:
            return IPS_ISYSBD3;
        case IPS_SYSGAL:
            return IPS_ISYSGAL;
        case IPS_SYSQZS:
            return IPS_ISYSQZS;
        case IPS_SYSIRN:
            return IPS_ISYSIRN;
        case IPS_SYSLEO:
            return IPS_ISYSLEO;
        }

        return -1;
    }

    /**
     * @brief       Compute the rotation matrix from ENU frame to ECEF frame
     * @note        The lat and lon should be rad instead of deg
     *
     * @param[in]   double      lat      latitude [rad]
     * @param[in]   double      lon      lontitude [rad]
     *
     * @return      Eigen::Matrix3d       the rotation matrix from n to e
     */
    Eigen::Matrix3d ComputeRotMat_ENU2ECEF(const double lat, const double lon)
    {
        Eigen::Matrix3d RotMat = Eigen::Matrix3d::Identity();

        double sinp = sin(lat), cosp = cos(lat), sinl = sin(lon), cosl = cos(lon);
        RotMat(0, 0) = -sinl, RotMat(0, 1) = -sinp * cosl, RotMat(0, 2) = cosp * cosl;
        RotMat(1, 0) = cosl, RotMat(1, 1) = -sinp * sinl, RotMat(1, 2) = cosp * sinl;
        RotMat(2, 0) = 0.0, RotMat(2, 1) = cosp, RotMat(2, 2) = sinp;

        return RotMat;
    }

    /**
     * @brief       Convert the position from ECEF to LLH
     * @note        The lat and lon should be rad instead of deg
     *
     * @param[in]   double[]      XYZ      position in ECEF [m]
     * @param[out]  double[]      LLH      position in LLH [rad,rad,m]
     *
     * @return
     */
    void XYZ2LLH(const double XYZ[3], double LLH[3])
    {
        double a = gs_WGS84_a, e2 = gs_WGS84_e2;
        double X = XYZ[0], Y = XYZ[1], Z = XYZ[2];
        double r2 = X * X + Y * Y;
        double z = 0.0, zk = 0.0, v = a, sinp = 0.0;

        for (z = Z, zk = 0.0; fabs(z - zk) >= 1E-4;)
        {
            zk = z;
            sinp = z / sqrt(r2 + z * z);
            v = a / sqrt(1.0 - e2 * sinp * sinp);
            z = Z + v * e2 * sinp;
        }

        LLH[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (Z > 0.0 ? PI / 2.0 : -PI / 2.0);
        LLH[1] = r2 > 1E-12 ? atan2(Y, X) : 0.0;
        LLH[2] = sqrt(r2 + z * z) - v;
    }

    /**
     * @brief       Convert the position from LLH to ECEF
     * @note        The lat and lon should be rad instead of deg
     *
     * @param[in]   double[]      LLH      position in LLH [rad,rad,m]
     * @param[out]  double[]      XYZ      position in ECEF [m]
     *
     * @return
     */
    void LLH2XYZ(const double LLH[3], double XYZ[3])
    {
        double a = gs_WGS84_a, e2 = gs_WGS84_e2;
        double sinp = sin(LLH[0]), cosp = cos(LLH[0]), sinl = sin(LLH[1]), cosl = cos(LLH[1]);
        double v = a / sqrt(1.0 - e2 * sinp * sinp);

        XYZ[0] = (v + LLH[2]) * cosp * cosl;
        XYZ[1] = (v + LLH[2]) * cosp * sinl;
        XYZ[2] = (v * (1.0 - e2) + LLH[2]) * sinp;
    }

    /**
     * @brief       Convert the string format time (yyyy mm dd hh mm ss) to GPS time
     * @note        the "yyyy mm dd hh mm ss" should be in the GPS time system
     *
     * @param[in]   s           char     time in string format ("... yyyy mm dd hh mm ss ...")
     * @param[in]   iPos        int      start location of string
     * @param[in]   nCount      int      length of string
     *
     * @return      GPSTIME
     *
     */
    IPS_GPSTIME str2time(const char *s, int iPos, int nCount)
    {
        IPS_GPSTIME gt;

        double ep[6] = {0.0};
        char str[256], *p = str;

        if (iPos < 0 || (int)strlen(s) < iPos || sizeof(str) - 1 < iPos)
            return gt;

        for (s += iPos; *s && --nCount >= 0;)
            *p++ = *s++;
        *p = '\0';

        if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6)
            return gt;

        if (ep[0] < 100.0)
            ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;

        gt = ymdhms2gps(IPS_YMDHMS(ep));
        if (gt.GPSWeek > 9999)
            gt.GPSWeek = -1;
        return gt;
    }

    /**
     * @brief       Convert the YMDHMS to GPS time
     * @note
     *
     * @param[in]   YMDHMS      t      year/month/day/hour/minut/second
     * @param[out]
     *
     * @return      GPSTIME
     */
    IPS_GPSTIME ymdhms2gps(IPS_YMDHMS t)
    {
        int year = t.year;
        int month = t.month;
        int mday = t.day;
        int hour = t.hour;
        int minute = t.min;
        double second = t.sec;

        long leap = (year % 4 == 0);
        long yday = month_day[leap][month - 1] + mday;
        long mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + yday - 1 + JAN11901;
        int gps_week = (mjd - JAN61980) / 7;
        double fmjd = second + minute * 60 + hour * 3600;
        double sec_of_week = ((mjd - JAN61980) - gps_week * 7) * SECPERDAY + fmjd;

        IPS_GPSTIME ot(gps_week, sec_of_week);
        return ot;
    }

    /**
     * @brief       Convert the GPS time to YMDHMS
     * @note
     *
     * @param[in]   IPS_GPSTIME      t      GPS Time
     * @param[out]
     *
     * @return      YMDHMS      year/month/day/hour/minut/second
     */
    IPS_YMDHMS gps2ymdhms(IPS_GPSTIME t)
    {
        int gps_week = t.GPSWeek;
        double sec_of_week = t.secsOfWeek + t.fracOfSec;
        long mjd = long(gps_week * 7 + sec_of_week / SECPERDAY + JAN61980);
        long days_fr_jan1_1901 = mjd - JAN11901;
        long num_four_yrs = days_fr_jan1_1901 / 1461;
        long years_so_far = 1901 + 4 * num_four_yrs;
        long days_left = days_fr_jan1_1901 - 1461 * num_four_yrs;
        long delta_yrs = days_left / 365 - days_left / 1460;
        long year = years_so_far + delta_yrs;
        long yday = days_left - 365 * delta_yrs + 1;
        double fmjd = fmod(sec_of_week, SECPERDAY);
        int hour = int(fmjd / 3600.0);
        int minute = int(fmjd / 60.0 - hour * 60.0);
        double second = fmod(sec_of_week, SECPERDAY) - hour * 3600.0 - minute * 60.;
        long leap = (year % 4 == 0);
        long guess = long(yday * 0.032);
        long more = ((yday - month_day[leap][guess + 1]) > 0);
        int month = guess + more + 1;
        int mday = yday - month_day[leap][guess + more];

        IPS_YMDHMS ot;
        ot.year = year;
        ot.month = month;
        ot.day = mday;
        ot.hour = hour;
        ot.min = minute;
        ot.sec = second;

        return ot;
    }

    /**
     * @brief       Convert the rtklib gtime to IPS GPSTime gt
     * @note
     *
     * @param[in]   gtime_t          src     rtklib gtime_t
     * @param[out]  IPS_GPSTIME      dst     IPS GPSTime
     *
     * @return
     */
    void ConvertTime(gtime_t src, IPS_GPSTIME *dst)
    {
        if (!dst)
            return;

        int Week = 0;
        double tow = time2gpst(src, &Week);
        dst->GPSWeek = Week;
        dst->secsOfWeek = (int)(tow);
        dst->fracOfSec = tow - dst->secsOfWeek;
    }

    /**
     * @brief       GPSTime minus
     * @note        It is suitable for GPSTime
     *
     * @param[in]   IPS_GPSTIME      gt1     IPS GPSTime
     * @param[out]  IPS_GPSTIME      gt2     IPS GPSTime
     *
     * @return
     */
    double MinusGPSTIME(IPS_GPSTIME gt1, IPS_GPSTIME gt2)
    {
        double lsec = (gt1.GPSWeek - gt2.GPSWeek) * 604800.0 + gt1.secsOfWeek - gt2.secsOfWeek;
        double t = lsec + gt1.fracOfSec - gt2.fracOfSec;
        return t;
    }

    /**
     * @brief       Convert rtklib PRN to IPS PRN
     * @note        It is suitable for GPSTime
     *
     * @param[in]   int      sat_rtk      rtklib prn
     * @param[out]
     *
     * @return      int      IPS PRN
     */
    int ConvertPrn(int sat_rtk)
    {
        int prn = 0, sys = 0, sat = 0;
        sys = satsys(sat_rtk, &sat);

        if (sys == SYS_GPS)
        {
            if (sat <= IPS_NSATGPS)
                prn = IPS_PRNGPS + sat;
        }
        else if (sys == SYS_GLO)
        {
            if (sat <= IPS_NSATGLO)
                prn = IPS_PRNGLO + sat;
        }
        else if (sys == SYS_GAL)
        {
            if (sat <= IPS_NSATGAL)
                prn = IPS_PRNGAL + sat;
        }
        else if (sys == SYS_QZS)
        {
            sat -= (MINPRNQZS - 1); // FIXME: add codes to debug (added by leiwh)
            if (sat <= IPS_NSATQZS)
                prn = IPS_PRNQZS + sat;
        }
        else if (sys == SYS_CMP)
        {
            if (sat <= IPS_NSATBD2)
                prn = IPS_PRNBD2 + sat;
            else if (sat > 18 && sat <= 18 + IPS_NSATBD3)
                prn = IPS_PRNBD3 + sat - 18;
        }

        return prn;
    }

    /**
     * @brief       Find the frequency and channel
     * @note
     *
     * @param[in]   int      sys      rtklib prn
     * @param[in]   char*    type[5]
     * @param[in]   obsd_t   obs      observation in rtklib
     *
     * @return      int      frequency
     */
    int FindFrqIndex(int sys, char (*type)[5], obsd_t obs)
    {
        int index = -1;
        char s[10] = "";
        if (sys == IPS_SYSGPS)
        {
            for (int f = 0; f < (NFREQ + NEXOBS); f++)
            {
                if (obs.code[f] == CODE_NONE)
                    continue;
                else if (obs.code[f] == CODE_L1C)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1P)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1W)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1Y)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1M)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1N)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1S)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1L)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L2C)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2D)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2S)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2L)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2X)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2P)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2W)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2Y)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2M)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2N)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L5I)
                    strcpy(s, "L5");
                else if (obs.code[f] == CODE_L5Q)
                    strcpy(s, "L5");
                else if (obs.code[f] == CODE_L5X)
                    strcpy(s, "L5");
                if (!strcmp(*type, s))
                {
                    index = f;
                    break;
                }
            }
        }
        else if (sys == IPS_SYSGLO)
        {
            for (int f = 0; f < (NFREQ + NEXOBS); f++)
            {
                if (obs.code[f] == CODE_NONE)
                    continue;
                else if (obs.code[f] == CODE_L1C)
                    strcpy(s, "G1");
                else if (obs.code[f] == CODE_L1P)
                    strcpy(s, "G1");
                else if (obs.code[f] == CODE_L2C)
                    strcpy(s, "G2");
                else if (obs.code[f] == CODE_L2P)
                    strcpy(s, "G2");
                if (!strcmp(*type, s))
                {
                    index = f;
                    break;
                }
            }
        }
        else if (sys == IPS_SYSBD2 || sys == IPS_SYSBD3)
        {
            for (int f = 0; f < (NFREQ + NEXOBS); f++)
            {
                if (obs.code[f] == CODE_NONE)
                    continue;
                else if (obs.code[f] == CODE_L2I)
                    strcpy(s, "B1I");
                else if (obs.code[f] == CODE_L2Q)
                    strcpy(s, "B1I");
                else if (obs.code[f] == CODE_L2X)
                    strcpy(s, "B1I");
                else if (obs.code[f] == CODE_L7I)
                    strcpy(s, "B2I");
                else if (obs.code[f] == CODE_L7Q)
                    strcpy(s, "B2I");
                else if (obs.code[f] == CODE_L7X)
                    strcpy(s, "B2I");
                else if (obs.code[f] == CODE_L6I)
                    strcpy(s, "B3I");
                else if (obs.code[f] == CODE_L6Q)
                    strcpy(s, "B3I");
                else if (obs.code[f] == CODE_L6X)
                    strcpy(s, "B3I");
                else if (obs.code[f] == CODE_L5D)
                    strcpy(s, "B2a");
                else if (obs.code[f] == CODE_L5P)
                    strcpy(s, "B2a");
                else if (obs.code[f] == CODE_L5X)
                    strcpy(s, "B2a");
                else if (obs.code[f] == CODE_L7D)
                    strcpy(s, "B2b");
                else if (obs.code[f] == CODE_L1D)
                    strcpy(s, "B1C");
                else if (obs.code[f] == CODE_L1P)
                    strcpy(s, "B1C");
                else if (obs.code[f] == CODE_L1X)
                    strcpy(s, "B1C");
                if (!strcmp(*type, s))
                {
                    index = f;
                    break;
                }
            }
        }
        else if (sys == IPS_SYSGAL)
        {
            for (int f = 0; f < (NFREQ + NEXOBS); f++)
            {
                if (obs.code[f] == CODE_NONE)
                    continue;
                else if (obs.code[f] == CODE_L1C)
                    strcpy(s, "E1");
                else if (obs.code[f] == CODE_L1A)
                    strcpy(s, "E1");
                else if (obs.code[f] == CODE_L1B)
                    strcpy(s, "E1");
                else if (obs.code[f] == CODE_L1X)
                    strcpy(s, "E1");
                else if (obs.code[f] == CODE_L1Z)
                    strcpy(s, "E1");
                else if (obs.code[f] == CODE_L6A)
                    strcpy(s, "E6");
                else if (obs.code[f] == CODE_L6B)
                    strcpy(s, "E6");
                else if (obs.code[f] == CODE_L6C)
                    strcpy(s, "E6");
                else if (obs.code[f] == CODE_L6X)
                    strcpy(s, "E6");
                else if (obs.code[f] == CODE_L6Z)
                    strcpy(s, "E6");
                else if (obs.code[f] == CODE_L7I)
                    strcpy(s, "E5b");
                else if (obs.code[f] == CODE_L7Q)
                    strcpy(s, "E5b");
                else if (obs.code[f] == CODE_L7X)
                    strcpy(s, "E5b");
                else if (obs.code[f] == CODE_L8I)
                    strcpy(s, "E5a_b");
                else if (obs.code[f] == CODE_L8Q)
                    strcpy(s, "E5a_b");
                else if (obs.code[f] == CODE_L8X)
                    strcpy(s, "E5a_b");
                else if (obs.code[f] == CODE_L5I)
                    strcpy(s, "E5a");
                else if (obs.code[f] == CODE_L5Q)
                    strcpy(s, "E5a");
                else if (obs.code[f] == CODE_L5X)
                    strcpy(s, "E5a");
                if (!strcmp(*type, s))
                {
                    index = f;
                    break;
                }
            }
        }
        else if (sys == IPS_SYSQZS)
        {
            for (int f = 0; f < (NFREQ + NEXOBS); f++)
            {
                if (obs.code[f] == CODE_NONE)
                    continue;
                else if (obs.code[f] == CODE_L1C)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1S)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1L)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L1X)
                    strcpy(s, "L1");
                else if (obs.code[f] == CODE_L6S)
                    strcpy(s, "LEX");
                else if (obs.code[f] == CODE_L6L)
                    strcpy(s, "LEX");
                else if (obs.code[f] == CODE_L6X)
                    strcpy(s, "LEX");
                else if (obs.code[f] == CODE_L2S)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2L)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L2X)
                    strcpy(s, "L2");
                else if (obs.code[f] == CODE_L5I)
                    strcpy(s, "L5");
                else if (obs.code[f] == CODE_L5Q)
                    strcpy(s, "L5");
                else if (obs.code[f] == CODE_L5X)
                    strcpy(s, "L5");
                if (!strcmp(*type, s))
                {
                    index = f;
                    break;
                }
            }
        }
        return index;
    }

    /**
     * @brief       Sort the GNSS observations data by GNSS PRN
     * @note
     *
     * @param[in]   IPS_OBSDATA *     src       observation data in IPS struct
     * @param[out]
     *
     * @return
     */
    void SortGNSSObs_IPSStruct(IPS_OBSDATA *src)
    {
        if (!src || src->nsat <= 1)
            return;

        std::sort(std::begin(src->obs), std::end(src->obs), [](const IPS_OBSDATA_t &obs1, const IPS_OBSDATA_t &obs2)
                  { return obs1.prn < obs2.prn; });
    }

    /**
     * @brief       Convert the GNSS observation data from rtklib struct to IPS struct
     * @note        It is used to process observation data in one epoch
     *
     * @param[in]   obsd_t *          src       observation data in rtklib struct
     * @param[in]   int               n         satellite number in rtklib
     * @param[out]  IPS_OBSDATA *     dst       observation data in IPS struct
     *
     * @return
     */
    void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, IPS_OBSDATA *dst)
    {
        if (!src || !dst)
            return;

        ///< 1. Prepare variables
        int index, sys = 0;
        char(*frq)[5] = NULL;
        char gs_strFrq_tmp[NFREQ][5] = {'\0'};
        IPS_GPSTIME gt;

        ConvertTime(src[0].time, &gt);
        if (MinusGPSTIME(gt, dst->gt) == 0.0)
            return;

        // clear the old data body
        dst->gt = gt;
        dst->flag = 0;
        dst->nsat = 0;
        memset(dst->ngnss, 0, sizeof(int) * IPS_NSYS);
        dst->obs.clear();

        for (int i = 0; i < n; i++)
        {
            if (dst->nsat >= MAXOBS)
                break;

            IPS_OBSDATA_t iobs;
            iobs.prn = ConvertPrn(src[i].sat);
            if (iobs.prn > IPS_NSATMAX || iobs.prn < 1)
                continue;

            for (int f = 0; f < NFREQ; f++)
            {
                std::memset(gs_strFrq_tmp[f], '\0', sizeof(gs_strFrq_tmp[f]));
            }

            satprn2no(iobs.prn, &sys);
            if (sys == IPS_SYSGPS)
            {
                dst->ngnss[0]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGPSFrq[f].c_str(), gnss_common::gs_strGPSFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSGLO)
            {
                dst->ngnss[2]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGLOFrq[f].c_str(), gnss_common::gs_strGLOFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSBD2)
            {
                dst->ngnss[2]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strBD2Frq[f].c_str(), gnss_common::gs_strBD2Frq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSBD3)
            {
                dst->ngnss[3]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strBD3Frq[f].c_str(), gnss_common::gs_strBD3Frq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSGAL)
            {
                dst->ngnss[4]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strGALFrq[f].c_str(), gnss_common::gs_strGALFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else if (sys == IPS_SYSQZS)
            {
                dst->ngnss[4]++;
                for (int f = 0; f < NFREQ; f++)
                    std::memcpy(gs_strFrq_tmp[f], gnss_common::gs_strQZSFrq[f].c_str(), gnss_common::gs_strQZSFrq[f].size() + 1);
                frq = gs_strFrq_tmp;
            }
            else
                continue;

            for (int f = 0; f < NFREQ; f++)
            {
                index = FindFrqIndex(sys, frq + f, src[i]);
                if (index < 0)
                    continue;
                iobs.P[f] = src[i].P[index];
                iobs.L[f] = src[i].L[index];
                iobs.D[f] = src[i].D[index];
                iobs.LLI[f] = src[i].LLI[index] & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK);
                iobs.S[f] = (float)(src[i].SNR[index] * SNR_UNIT);
            }

            dst->obs.push_back(iobs);
            dst->nsat++;
        }
        SortGNSSObs_IPSStruct(dst);
        frq = NULL;

        return;
    }

    /**
     * @brief       Convert rtklib eph data to IPS eph data
     * @note        GLONASS eph is not considered
     *
     * @param[in]   eph_t*           src     rtklib eph data
     * @param[out]  IPS_GPSEPH*      n       IPS eph data
     *
     * @return
     */
    void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, IPS_GPSEPH *dst)
    {
        ConvertTime(src->toe, &dst->toe);
        ConvertTime(src->toc, &dst->toc);

        dst->prn = ConvertPrn(src->sat);
        dst->iode = src->iode;
        dst->iodc = src->iodc;
        dst->sva = src->sva;
        dst->svh = src->svh;
        dst->week = src->week;
        dst->code = src->code;
        dst->A = src->A;
        dst->e = src->e;
        dst->i0 = src->i0;
        dst->OMG0 = src->OMG0;
        dst->omg = src->omg;
        dst->M0 = src->M0;
        dst->deln = src->deln;
        dst->OMGd = src->OMGd;
        dst->idot = src->idot;
        dst->crc = src->crc;
        dst->crs = src->crs;
        dst->cuc = src->cuc;
        dst->cus = src->cus;
        dst->cic = src->cic;
        dst->cis = src->cis;
        dst->toes = src->toes;
        dst->f0 = src->f0;
        dst->f1 = src->f1;
        dst->f2 = src->f2;

        for (int i = 0; i < 4; i++)
        {
            dst->tgd[i] = src->tgd[i];
        }
    }

    /**
     * @brief       Convert rtklib nav data to IPS eph data
     * @note        GLONASS eph is not considered
     *
     * @param[in]   nav_t*           src     rtklib eph data
     * @param[out]  IPS_GPSEPH*      n       IPS eph data
     *
     * @return
     */
    void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, IPS_GPSEPH *dst)
    {
        if (dst == NULL)
            return;

        for (int i = 0; i < (src->n - src->ng); i++)
        {
            int prn = ConvertPrn(src->eph[i].sat);
            if (prn < 1 || prn > IPS_NSATMAX)
                continue;

            IPS_GPSTIME gt;
            ConvertTime(src->eph[i].toe, &gt);
            double dt = MinusGPSTIME(gt, dst[prn - 1].toe);
            if (dt <= 0.0)
                continue;

            Convert_GNSSEphStruct_RTKLIB2IPS(&src->eph[i], &dst[prn - 1]);
        }
    }

}

/**
 * @brief       Convert the GNSS observation data from IPS struct to RobotGVINS struct
 * @note        It is used to process observation data in one epoch
 *
 * @param[in]   IPS_OBSDATA *           ipsdata         observation data in IPS struct
 * @param[in]   RobotGVINS_GNSSObs      robotdata       observation data in RobotGVINS struct
 *
 * @return
 */
void Convert_GNSSObsStruct_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, rosbagio::RobotGVINS_GNSSObs &robotdata)
{
    if (ipsdata == NULL)
        return;

    // timestamp
    robotdata.header.stamp = ros::Time(ipsdata->pubtime);
    // robotdata.header.stamp = ros::Time(ipsdata->gt.GPSWeek * 604800 + ipsdata->gt.secsOfWeek + ipsdata->gt.fracOfSec);

    // observation info
    robotdata.flag = ipsdata->flag;
    robotdata.nsat = ipsdata->nsat;
    for (int i = 0; i < IPS_NSYS; i++)
        robotdata.ngnss.push_back(ipsdata->ngnss[i]);

    // observation data for each satellite
    for (int i = 0; i < ipsdata->obs.size(); i++)
    {
        rosbagio::RobotGVINS_GNSSSat sat_msg;
        sat_msg.prn = ipsdata->obs.at(i).prn;
        for (int f = 0; f < NFREQ; f++)
        {
            sat_msg.cp_meas.push_back(ipsdata->obs.at(i).L[f]);
            sat_msg.pr_meas.push_back(ipsdata->obs.at(i).P[f]);
            sat_msg.do_meas.push_back(ipsdata->obs.at(i).D[f]);
            sat_msg.sig_cno.push_back(ipsdata->obs.at(i).S[f]);
            sat_msg.code.push_back(ipsdata->obs.at(i).code[f]);
            sat_msg.SNR.push_back(ipsdata->obs.at(i).SNR[f]);
            sat_msg.LLI.push_back(ipsdata->obs.at(i).LLI[f]);
            sat_msg.cs.push_back(ipsdata->obs.at(i).cs[f]);
            sat_msg.P_TGD.push_back(ipsdata->obs.at(i).P_TGD[f]);
            sat_msg.SMP.push_back(ipsdata->obs.at(i).SMP[f]);
        }
        robotdata.obsdata.push_back(sat_msg);
    }
}

/**
 * @brief       Convert the GNSS ephemeris data from IPS struct to RobotGVINS struct
 * @note        It is used to process ephemeris data for only one satellite
 *
 * @param[in]   IPS_GPSEPH *            ipsdata         ephemeris data in IPS struct
 * @param[in]   RobotGVINS_GNSSEph      robotdata       ephemeris data in RobotGVINS struct
 *
 * @return
 */
void Convert_GNSSEphStruct_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, rosbagio::RobotGVINS_GNSSEph &robotdata)
{
    if (ipsdata == NULL)
        return;

    // get the ephemeris system
    int sys = IPS_SYSNON;
    int prn = ipsdata->prn;                               // GNSS prn in IPS program
    int sat = gnss_common::satprn2no(prn, &sys);          // GNSS prn for each system
    robotdata.header.stamp = ros::Time(ipsdata->pubtime); // the timestamp to publish ros message

    // get the data body
    robotdata.prn = ipsdata->prn;
    robotdata.iode = ipsdata->iode;
    robotdata.iodc = ipsdata->iodc;
    robotdata.sva = ipsdata->sva;
    robotdata.svh = ipsdata->svh;
    robotdata.week = ipsdata->week;
    robotdata.code = ipsdata->code;
    robotdata.flag = ipsdata->flag;
    robotdata.toe = ipsdata->toe.GPSWeek * 604800 + ipsdata->toe.secsOfWeek + ipsdata->toe.fracOfSec;
    robotdata.toc = ipsdata->toc.GPSWeek * 604800 + ipsdata->toc.secsOfWeek + ipsdata->toc.fracOfSec;
    robotdata.ttr = ipsdata->ttr.GPSWeek * 604800 + ipsdata->ttr.secsOfWeek + ipsdata->ttr.fracOfSec;
    robotdata.eph_A = ipsdata->A;
    robotdata.eph_e = ipsdata->e;
    robotdata.i0 = ipsdata->i0;
    robotdata.OMG0 = ipsdata->OMG0;
    robotdata.omg = ipsdata->omg;
    robotdata.M0 = ipsdata->M0;
    robotdata.deln = ipsdata->deln;
    robotdata.OMGd = ipsdata->OMGd;
    robotdata.idot = ipsdata->idot;
    robotdata.crc = ipsdata->crc;
    robotdata.crs = ipsdata->crs;
    robotdata.cuc = ipsdata->cuc;
    robotdata.cus = ipsdata->cus;
    robotdata.cic = ipsdata->cic;
    robotdata.cis = ipsdata->cis;
    robotdata.toes = ipsdata->toes;
    robotdata.fit = ipsdata->fit;
    robotdata.f0 = ipsdata->f0;
    robotdata.f1 = ipsdata->f1;
    robotdata.f2 = ipsdata->f2;
    for (int i = 0; i < 4; i++)
        robotdata.tgd[i] = ipsdata->tgd[i];
}