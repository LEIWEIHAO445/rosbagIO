/**
 * @brief
 */

#include "rosbagIO_header.h"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("arguments are invalied!\n");
        return false;
    }

    // TODO: extract imu data and write to the bag
    // step 1: extract imu data from the ros bag file
    std::string imutopic = "/imu/data";
    std::list<sensor_msgs::Imu> imudatas(0);
    std::string infilepath(argv[1]);
    extract_IMUdata_ROSFormat(infilepath.c_str(), imutopic, imudatas); // extract as ros standard format
    // step 2: write imu data to the ros bag file
    imutopic = "/imu/data";
    std::string outfilepath(argv[2]);
    write_IMUdata_ROSBag(outfilepath.c_str(), imutopic, imudatas, 1); // write as ros standard format

    // TODO: extract image data and write to the bag
    // // step 1: extract image data from the ros bag file
    // std::string imgtopic = "/camera/lowres/image";
    // std::list<sensor_msgs::Image> imgdatas;
    // std::string infilepath(argv[1]);
    // extract_ImageData_ROSFormat(infilepath.c_str(), imgtopic, imgdatas); // extract as ros standard format
    // // step 2: write image data to the ros bag file
    // imgtopic = "/camera/lowres/image";
    // std::string outfilepath(argv[2]);
    // write_ImageData_ROSBag(outfilepath.c_str(), imgtopic, imgdatas, 2); // write as ros standard format

    // // TODO: extract gnss solution data and write to bag
    // // 1. extract gnss solutiion
    // std::string infilepath(argv[1]);
    // std::list<rosbagio::RobotGVINS_GNSSSol> gnsssol_datas(0);
    // extract_GNSSSolData_IPSPOSFMT(infilepath.c_str(), gnsssol_datas);
    // // extract_GNSSSolData_KAIST_VRSGPS(infilepath.c_str(), KAIST_gnsssol_topic, gnsssol_datas);
    // // 2. write gnss solution data
    // std::string outfilepath(argv[2]);
    // std::string gnsssol_topic = "/gnss/solution";
    // std::string gimu_topic = "/gnss/solution";
    // // write_GNSSSolData_ROSFormat2ROSBag(outfilepath.c_str(), KAIST_gnsssol_topic, VisionRTK2_imu_topic, gnsssol_datas, 2);  // write as ros standard format
    // // write_GNSSSolData_RobotGVINS2ROSBag(outfilepath.c_str(), KAIST_gnsssol_topic, VisionRTK2_imu_topic, gnsssol_datas, 2); // write as RobotGVINS format
    // write_GNSSSolData_RobotGVINS2ROSBag(outfilepath.c_str(), gnsssol_topic, VisionRTK2_imu_topic, gnsssol_datas, 2);

    // TODO: extract gnss observations and ephemeris data and write to the rosbag file
    // // step 1: extract gnss raw data from the ros bag file (Vision-RTK2 file format)
    // std::string infilepath(argv[1]);
    // std::string rostopic = "/gnss1/raw";
    // std::string imu_topic = "/imu/data";
    // std::list<gnss_common::IPS_OBSDATA> gnss_obsdata(0);
    // std::list<gnss_common::IPS_GPSEPH> gnss_ephdata(0);
    // extract_GNSSRawData_VisionRTK2(infilepath.c_str(), NULL, rostopic, gnss_obsdata, gnss_ephdata);
    // // // step 2: write the GNSS observations data to bag file with RobotGVINS format
    // std::string outfilepath(argv[2]); // the filepath to write data
    // std::string gnss_obstopic_rove = "/gnss/obs/rove";
    // write_GNSSObsData_IPSStruct2ROSBag(outfilepath.c_str(), gnss_obstopic_rove, imu_topic, gnss_obsdata, 2);
    // // step 3: write the GNSS navigation data to bag file with RobotGVINS format
    // std::string gnss_ephtopic_rove = "/gnss/ephdata";
    // write_GNSSEphData_IPSStruct2ROSBag(outfilepath.c_str(), gnss_ephtopic_rove, imu_topic, gnss_ephdata, 2);

    // TODO: read gnss rinex format observation file and write to ros bag file
    // // step 1: use the IPS function and store all observations data in all epochs in IPS struct
    // std::string infilepath(argv[1]);
    // std::list<gnss_common::IPS_OBSDATA> gnss_obsdata(0);
    // extract_GNSSObsData_RINEX3_IPSVersion(infilepath.c_str(), gnss_obsdata);
    // // step 2: write the GNSS observations data to bag file with custmoized format
    // std::string outfilepath(argv[2]);
    // std::string gnss_obstopic_base = "/gnss/obs/base";
    // std::string imu_topic = "/imu/data";
    // write_GNSSObsData_IPSStruct2ROSBag(outfilepath.c_str(), gnss_obstopic_base, imu_topic, gnss_obsdata, 2);

    return 0;
}