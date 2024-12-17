/**
 * @brief
 */

#ifndef __ROSBAGINO_HEADER_H__
#define __ROSBAGINO_HEADER_H__

#include <iostream>
#include <filesystem>
#include <fstream>
#include <dirent.h>
#include <unistd.h>
#include <vector>
#include <list>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/foreach.hpp>
#include <VisionRTK2_GNSSRaw.h>
#include <VisionRTK2_GNSSEpoch.h>
#include <VisionRTK2_GNSSRaw.h>
#include <VisionRTK2_GNSSSat.h>
#include <VisionRTK2_GNSSSig.h>
#include <VIsionRTK2_GNSSStatus.h>
#include <RobotGVINS_GNSSSol.h>
#include <RobotGVINS_GNSSSat.h>
#include <RobotGVINS_GNSSObs.h>
#include <RobotGVINS_GNSSEph.h>
#include <GVINS_GNSSMeasMsg.h>
#include <GVINS_GNSSObsMsg.h>
#include <GVINS_GNSSPVTSolnMsg.h>
#include <GVINS_GNSSTimeMsg.h>
#include <KAIST_VRSGPS.h>
#include <KAIST_XsensIMU.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/NavSatFix.h"
#include "rtklib.h"

/***********************************************************************************************
 * The Definition of Constant Variables
 ***********************************************************************************************/

// common
#define foreach BOOST_FOREACH
#define ZeroStruct(x, type) (memset(&x, 0, sizeof(type))) ///< clear struct

// math
#define IPS_MAXSIZE 1024                            ///< the max size of variables
#define IPS_MAXANT 64                               ///< max length of station name/antenna type
#define SQR(x) ((x) * (x))                          ///< x*x
#define IPS_EPSILON 2.2204460492503131e-016         ///< epsilon
#define IPS_D2R (0.0174532925199432957692369076849) ///< deg to rad
#define IPS_R2D (57.295779513082322864647721871734) ///< rad to deg

// gnss
#define IPS_SYSNON 0x00
#define IPS_SYSGPS 0x01
#define IPS_SYSGLO 0x02
#define IPS_SYSBD2 0x04
#define IPS_SYSBD3 0x08
#define IPS_SYSGAL 0x10
#define IPS_SYSQZS 0x20
#define IPS_SYSIRN 0x40
#define IPS_SYSLEO 0x80
#define IPS_SYSALL (IPS_SYSGPS | IPS_SYSGLO | IPS_SYSBD2 | IPS_SYSBD3 | IPS_SYSGAL | IPS_SYSQZS)
#define IPS_ISYSNON -1
#define IPS_ISYSGPS 0
#define IPS_ISYSGLO 1
#define IPS_ISYSBD2 2
#define IPS_ISYSBD3 3
#define IPS_ISYSGAL 4
#define IPS_ISYSQZS 5
#define IPS_ISYSIRN 6
#define IPS_ISYSLEO 7
#define IPS_NSYS 6
#define IPS_MAXOBS 200

#define IPS_PRNGPS 0
#define IPS_NSATGPS 32
#define IPS_PRNGLO (IPS_PRNGPS + IPS_NSATGPS)
#define IPS_NSATGLO 27
#define IPS_PRNBD2 (IPS_PRNGLO + IPS_NSATGLO)
#define IPS_NSATBD2 18
#define IPS_PRNBD3 (IPS_PRNBD2 + IPS_NSATBD2)
#define IPS_NSATBD3 45 ///< 3GEO + 3IGSO + 24MEO
#define IPS_PRNGAL (IPS_PRNBD3 + IPS_NSATBD3)
#define IPS_NSATGAL 36
#define IPS_PRNQZS (IPS_PRNGAL + IPS_NSATGAL)
#define IPS_NSATQZS 8
#define IPS_NSATMAX (IPS_NSATGPS + IPS_NSATGLO + IPS_NSATBD2 + IPS_NSATBD3 + IPS_NSATGAL + IPS_NSATQZS)

namespace gnss_common
{
    static const double gs_WGS84_a = 6378137.0;            ///< earth semimajor axis (WGS84) (m)
    static const double gs_WGS84_b = 6356752.31425;        ///< earth semimajor axis (WGS84) (m)
    static const double gs_WGS84_FE = 1.0 / 298.257223563; ///< earth flattening (WGS84)
    static const double gs_WGS84_e2 = 2 * gs_WGS84_FE - SQR(gs_WGS84_FE);

    static const bool gs_bSwitchGNSSFrq = false;
    static const std::string gs_strGPSFrq[NFREQ] = {"L1", "L2", "L5"};
    static const std::string gs_strGLOFrq[NFREQ] = {"G1", "G2", "G3"};
    static const std::string gs_strBD2Frq[NFREQ] = {"B1I", "B2I", "B3I"};
    static const std::string gs_strBD3Frq[NFREQ] = {"B1I", "B2I", "B3I"};
    static const std::string gs_strGALFrq[NFREQ] = {"E1", "E5b", "E5a"};
    static const std::string gs_strQZSFrq[NFREQ] = {"L1", "L2", "L5"};

    const static long JAN61980 = 44244; // gps time reference
    const static long JAN12006 = 53736; // bds time reference
    const static long AG221999 = 51412; // GAL time reference
    const static long JAN11901 = 15385;
    const static double SECPERDAY = 86400.0;

    const static long month_day[2][13] = {
        {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365},
        {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366}};
}

// time system
const int GPS_LINUX_TIME = 315964800;
const int LEAP_SECOND = 18;
const int GPS_BDS_WEEK = 1356;
const double GPS_BDS_SECOND = 14.0;

// ros topic
static std::string VisionRTK2_imu_topic = "/imu/data";
static std::string VisionRTK2_image_topic = "/camera/lowres/image";
static std::string VisionRTK2_gnssraw_topic = "/gnss1/raw";
static std::string KAIST_imu_topic = "/xsens_imu_data";
static std::string KAIST_leftimage_topic = "/stereo/left/image_raw";
static std::string KAIST_gnsssol_topic = "/vrs_gps_data";
static std::string RobotGVINS_imu_topic = "/imu/data";
static std::string RobotGVINS_gnsssol_topic = "/gnss/solution";

/***********************************************************************************************
 * The Definition of Class and Struct
 ***********************************************************************************************/

namespace gnss_common
{
    struct IPS_YMDHMS
    {
        int year;
        int month;
        int day;
        int hour;
        int min;
        double sec;

        IPS_YMDHMS()
        {
            year = 2000;
            month = day = 1;
            hour = min = 0;
            sec = 0.0;
        }

        IPS_YMDHMS(int y, int m, int d, int h, int n, double s)
        {
            year = y;
            month = m;
            day = d;
            hour = h;
            min = n;
            sec = s;
        }

        IPS_YMDHMS(const double *ep)
        {
            year = (int)ep[0];
            month = (int)ep[1];
            day = (int)ep[2];
            hour = (int)ep[3];
            min = (int)ep[4];
            sec = ep[5];
        }
    };

    struct IPS_GPSTIME ///< GPS time
    {
        int GPSWeek;
        int secsOfWeek;
        double fracOfSec;

        IPS_GPSTIME()
        {
            GPSWeek = -1;
            secsOfWeek = 0;
            fracOfSec = 0.0;
        }

        IPS_GPSTIME(int week, double sec)
        {
            GPSWeek = week;
            secsOfWeek = int(sec);
            fracOfSec = sec - secsOfWeek;
            if (fracOfSec < IPS_EPSILON)
            {
                fracOfSec = 0.0;
            }
        }

        IPS_GPSTIME(int week, int isec, double fsec)
        {
            GPSWeek = week;
            secsOfWeek = isec;
            fracOfSec = fsec;
            if (fracOfSec < IPS_EPSILON)
            {
                fracOfSec = 0.0;
            }
        }
    };

    struct IPS_OBSHEAD ///< GNSS observation header
    {
        char bValid[13];
        char antType[IPS_MAXANT]; ///< antenna type number
        char recType[IPS_MAXANT]; ///< receiver type descriptor
        double XYZ[3];            ///< station position (ecef) (m)
        double ant[3];            ///< antenna position delta (e/n/u or x/y/z) (m)
        double dt;                ///< data sampling rate

        IPS_OBSHEAD()
        {
            ZeroStruct(*this, IPS_OBSHEAD);
        }
    };

    struct IPS_OBSDATA_t ///< Obs observatino body
    {
        int prn;
        double L[NFREQ];          ///< observation data carrier-phase (cycle)
        double P[NFREQ];          ///< observation data pseudorange (m)
        double D[NFREQ];          ///< observation data doppler frequency (Hz)
        float S[NFREQ];           ///< signal strength
        char code[NFREQ][3];      ///< P code type (channel)
        unsigned char SNR[NFREQ]; ///< signal strength (0.25 dBHz)
        unsigned char LLI[NFREQ]; ///< loss of lock indicator
        double cs[NFREQ];         ///< simulated cycle slip
        double P_TGD[NFREQ];      ///< observation data pseudorange after TGD correction(m)
        double SMP[NFREQ];        ///< Carrier Smoothing of Code Pseudoranges

        IPS_OBSDATA_t()
        {
            ZeroStruct(*this, IPS_OBSDATA_t);
        }

        bool operator==(IPS_OBSDATA_t data)
        {
            return prn == data.prn;
        }
    };

    struct IPS_OBSDATA // observation data of all epochs
    {
        double pubtime;                 ///< publish time (for ros)
        IPS_GPSTIME gt;                 ///< gps time
        int flag;                       ///< 0: avaiable
        int nsat;                       ///< satellites number
        int ngnss[IPS_NSYS];            ///< satellites number of each system
        std::vector<IPS_OBSDATA_t> obs; ///< observations data of each satellite

        IPS_OBSDATA()
        {
            gt.GPSWeek = -1;
            gt.secsOfWeek = -1;
            gt.fracOfSec = 0.0;
            pubtime = 0.0;
            nsat = 0;
            flag = 0;
            for (int i = 0; i < IPS_NSYS; i++)
                ngnss[i] = 0;
        }
    };

    typedef struct tagGPSEPH ///< GPS broadcast ephemeris type
    {
        double pubtime;                                   ///< publish time (for ros)
        int prn;                                          ///< satellite number
        int iode, iodc;                                   ///< IODE,IODC
        double sva;                                       ///< SV accuracy (m)
        int svh;                                          ///< SV health (0:ok)
        int week;                                         ///< GPS/QZS: gps week, GAL: galileo week
        int code;                                         ///< GPS/QZS: code on L2, GAL: data sources
        int flag;                                         ///< GPS/QZS: L2 P data flag
        IPS_GPSTIME toe, toc, ttr;                        ///< Toe,Toc,T_trans
        double A, e, i0, OMG0, omg, M0, deln, OMGd, idot; ///< SV orbit parameters
        double crc, crs, cuc, cus, cic, cis;              ///< SV orbit parameters
        double toes;                                      ///< Toe (s) in week
        double fit;                                       ///< fit interval (h)
        double f0, f1, f2;                                ///< SV clock parameters (af0,af1,af2)
        double tgd[4];                                    ///< group delay parameters
                                                          ///< tgd[0]: L1/L2, B1/B3, tgd[1]: B2/B3

        tagGPSEPH()
        {
            ZeroStruct(*this, tagGPSEPH);
        }

    } IPS_GPSEPH, IPS_BDSEPH, IPS_GALEPH, IPS_QZSEPH;
}

/***********************************************************************************************
 * The Definition of Function
 ***********************************************************************************************/

/**
 * @brief       Copy the string from the src to the dst
 */
void xstrmid(const char *src, const int nPos, const int nCount, char *dst);

// gnss function
namespace gnss_common
{
    /**
     * @brief       Convert the second to GPSWeek and GPSSecond
     */
    IPS_GPSTIME toGPSTIME(double sec);

    /**
     * @brief       Convert the customized PRN to GNSS PRN
     */
    int satprn2no(const int prn, int *sys);

    /**
     * @brief       Convert the customized PRN to GNSS PRN (Gxx)
     */
    std::string satprn2no(const int prn);

    /**
     * @brief       Convert the GNSS PRN to the customized PRN

     */
    int satno2prn(const char *no);

    /**
     * @brief       Find the index based on the system
     */
    int Sys2Index(int sys);

    /**
     * @brief       Convert the position from ECEF to LLH
     */
    void XYZ2LLH(const double XYZ[3], double LLH[3]);

    /**
     * @brief       Convert the position from LLH to ECEF
     */
    void LLH2XYZ(const double LLH[3], double XYZ[3]);

    /**
     * @brief       Compute the rotation matrix from ENU frame to ECEF frame
     */
    Eigen::Matrix3d ComputeRotMat_ENU2ECEF(const double lat, const double lon);

    /**
     * @brief       Convert the string format time (yyyy mm dd hh mm ss) to GPS time
     */
    IPS_GPSTIME str2time(const char *s, int iPos, int nCount);

    /**
     * @brief       Convert the YMDHMS to GPS time
     */
    IPS_GPSTIME ymdhms2gps(IPS_YMDHMS t);

    /**
     * @brief       Convert the GPS time to YMDHMS
     */
    IPS_YMDHMS gps2ymdhms(IPS_GPSTIME t);

    /**
     * @brief       Convert the rtklib gtime to IPS GPSTime gt
     */
    void ConvertTime(gtime_t src, IPS_GPSTIME *dst);

    /**
     * @brief       GPSTime minus
     */
    double MinusGPSTIME(IPS_GPSTIME gt1, IPS_GPSTIME gt2);

    /**
     * @brief       Convert rtklib PRN to IPS PRN
     */
    int ConvertPrn(int sat_rtk);

    /**
     * @brief       Find the frequency and channel
     */
    int FindFrqIndex(int sys, char (*type)[5], obsd_t obs);

    /**
     * @brief       Sort the GNSS observations data by GNSS PRN
     */
    void SortGNSSObs_IPSStruct(IPS_OBSDATA *src);

    /**
     * @brief       Convert the GNSS observation data from rtklib struct to IPS struct
     */
    void Convert_GNSSObsStruct_RTKLIB2IPS(const obsd_t *src, int n, IPS_OBSDATA *dst);

    /**
     * @brief       Convert rtklib eph data to IPS eph data
     */
    void Convert_GNSSEphStruct_RTKLIB2IPS(const eph_t *src, IPS_GPSEPH *dst);

    /**
     * @brief       Convert rtklib nav data to IPS eph data
     */
    void Convert_GNSSNavStruct_RTKLIB2IPS(const nav_t *src, IPS_GPSEPH *dst);

}

/**
 * @brief      extract image data from bag file (ros standard format)
 */
extern bool extract_ImageData_ROSFormat(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas);

/**
 * @brief      extract IMU data from bag file (ros standard format)
 */
extern bool extract_IMUdata_ROSFormat(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas);

/**
 * @brief      extract IMU data from bag file (KAIST Xsens format) and save as ros standard format
 */
extern bool extract_IMUdata_KAIST_XsensFormat(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas);

/**
 * @brief      extract GNSS solution data from the bag file (HKUST-Aerial-GVINS format) and save as RobotGVINS format
 */
extern bool extract_GNSSSolData_HKUSTGVINS(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas);

/**
 * @brief      extract GNSS solution data from the bag file (KAIST vrs_gps format) and save as RobotGVINS format
 */
extern bool extract_GNSSSolData_KAIST_VRSGPS(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas);

/**
 * @brief      extract GNSS solution data from the bag file (ros standard format) and save as RobotGVINS format
 */
extern bool extract_GNSSSolData_ROSFormat(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas);

/**
 * @brief      extract GNSS solution data from IPS .pos format file and save as RobotGVINS format
 */
extern bool extract_GNSSSolData_IPSPOSFMT(const char *pos_infilepath, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas);

/**
 * @brief      extract GNSS raw data from the bag file (Vision-RTK2 data format) and save as RobotGVINS format
 */
extern bool extract_GNSSRawData_VisionRTK2(const char *bag_infilepath, const char *gnssraw_outfilepath, const std::string &gnssraw_topic, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata);

/**
 * @brief      extract GNSS epoch data from the bag file (Vision-RTK2 data format)
 */
extern bool extract_GNSSEpochData_VisionRTK2(const char *bag_infilepath, const char *gnssepoch_outfilepath, const std::string &gnssepoch_topic);

/**
 * @brief      extract GNSS observation data from the rinex 3.0x file (IPS version) and save as IPS struct
 */
extern bool extract_GNSSObsData_RINEX3_IPSVersion(const char *rinex_infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata);

/**
 * @brief      write IMU data to bag file
 */
extern bool write_IMUdata_ROSBag(const char *bag_outfilepath, const std::string imu_topic, const std::list<sensor_msgs::Imu> &imudatas, int bagmode = 1);

/**
 * @brief      write Image data to bag file
 */
extern bool write_ImageData_ROSBag(const char *bag_outfilepath, const std::string img_topic, const std::list<sensor_msgs::Image> &imgdatas, int bagmode = 1);

/**
 * @brief      write GNSS observations data to ros bag file
 */
extern bool write_GNSSObsData_IPSStruct2ROSBag(const char *bag_outfilepath, const std::string gnssobs_topic, const std::string imu_topic, const std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, int bagmode = 1);

/**
 * @brief      write GNSS ephemeris data to ros bag file
 */
extern bool write_GNSSEphData_IPSStruct2ROSBag(const char *bag_outfilepath, const std::string gnss_ephtopic, const std::string imu_topic, const std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata, int bagmode = 1);

/**
 * @brief      write gnss solution data (RobotGVINS format) to ros bag file (rtos standard format)
 */
extern bool write_GNSSSolData_ROSFormat2ROSBag(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode = 1);

/**
 * @brief      write gnss solution data (RobotGVINS format) to ros bag file (RobotGVINS format)
 */
extern bool write_GNSSSolData_RobotGVINS2ROSBag(const char *bag_outfilepath, const std::string gnsssol_topic, const std::string imu_topic, const std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas, int bagmode = 1);

/**
 * @brief      write GNSS observation data to the file (IPS struct)
 */
extern bool write_GNSSObsData_IPSStruct(FILE *outfile, const gnss_common::IPS_OBSDATA *obsdata);

/**
 * @brief      write GNSS ephemeris data to the file (IPS struct)
 */
extern bool write_GNSSEphData_IPSStruct(FILE *outfile, const gnss_common::IPS_GPSEPH *ephdata);

/**
 * @brief       Convert the GNSS observation data from IPS struct to RobotGVINS struct
 */
extern void Convert_GNSSObsStruct_IPS2RobotGVINS(const gnss_common::IPS_OBSDATA *ipsdata, rosbagio::RobotGVINS_GNSSObs &robotdata);

/**
 * @brief       Convert the GNSS ephemeris data from IPS struct to RobotGVINS struct
 */
extern void Convert_GNSSEphStruct_IPS2RobotGVINS(const gnss_common::IPS_GPSEPH *ipsdata, rosbagio::RobotGVINS_GNSSEph &robotdata);

#endif