/**
 * @brief
 */

#include "rosbagIO_header.h"

/**
 * @brief      extract Image data from bag file (ros standard format)
 * @note
 *
 * @param[in]  char*      bag_infilepath      filepath to read data
 * @param[in]  string     img_topic           ros topic
 * @param[out] list       imgdatas            store image data of all epochs
 *
 * @return     bool       true      extract successfully
 *                        false     fail to extract
 */
extern bool extract_ImageData_ROSFormat(const char *bag_infilepath, const std::string &img_topic, std::list<sensor_msgs::Image> &imgdatas)
{
    // 1. Open the bag file to read image data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);
    if (!bag_in.isOpen())
    {
        printf("open ros bag file to read data unsuccessfully!\n");
        return false;
    }

    // 2. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(img_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 3. Read image data from the bag file and write
    foreach (rosbag::MessageInstance const m, view)
    {
        // get one image message
        sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();

        // get the data body
        sensor_msgs::Image one_imgdata = *image_msg;

        // convert the time timestamp from Linux time to GPS time
        double timestamp = one_imgdata.header.stamp.sec + one_imgdata.header.stamp.nsec * 1e-9;
        timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
        one_imgdata.header.stamp = ros::Time(timestamp);

        // store the message
        imgdatas.push_back(one_imgdata);

        // output cv::Mat to check the raw image
        // cv::Mat image_data = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        // cv::imwrite(output_filepath, image_data);
    }

    // close file
    bag_in.close();

    return true;
}

/**
 * @brief       extract IMU observations from ros bag file (ros standard format)
 * @note
 *
 * @param[in]   char*      bag_infilepath      filepath
 * @param[in]   string     imu_topic           topic
 * @param[out]  list       imudatas            imu raw data of all epochs
 *
 * @return      bool       true      extract successfully
 *                        false      fail to extract
 */
bool extract_IMUdata_ROSFormat(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas)
{
    // 1. Open the bag file to read imu data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);
    if (!bag_in.isOpen())
    {
        printf("open ros bag file to read data unsuccessfully!\n");
        return false;
    }

    // 2. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(imu_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 3. Read and store the imu data
    foreach (rosbag::MessageInstance const m, view)
    {
        if (m.instantiate<sensor_msgs::Imu>() != nullptr)
        {
            // get one imu message
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();

            // get the body data
            sensor_msgs::Imu one_imudata;
            one_imudata = *imu_msg;

            // convert the time timestamp from Linux time to GPS time
            double timestamp = one_imudata.header.stamp.sec + one_imudata.header.stamp.nsec * 1e-9;
            timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
            one_imudata.header.stamp = ros::Time(timestamp);

            // store the imu message
            imudatas.push_back(one_imudata);
        }
    }

    // close the file
    bag_in.close();

    return true;
}

/**
 * @brief      extract IMU data from bag file (KAIST Xsens format) and save as ros standard format
 * @note
 *
 * @param[in]   char*      bag_infilepath      filepath
 * @param[in]   string     imu_topic           topic
 * @param[out]  list       imudatas            imu raw data of all epochs
 *
 * @return      bool       true      extract successfully
 *                        false      fail to extract
 */
extern bool extract_IMUdata_KAIST_XsensFormat(const char *bag_infilepath, const std::string &imu_topic, std::list<sensor_msgs::Imu> &imudatas)
{
    // 1. Open the bag file to read imu data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);
    if (!bag_in.isOpen())
    {
        printf("open ros bag file to read data unsuccessfully!\n");
        return false;
    }

    // 2. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(imu_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 3. Read and store the imu data
    foreach (rosbag::MessageInstance const m, view)
    {
        if (m.instantiate<rosbagio::KAIST_XsensIMU>() != nullptr)
        {
            // get one imu message
            rosbagio::KAIST_XsensIMU::ConstPtr imu_msg = m.instantiate<rosbagio::KAIST_XsensIMU>();

            // store the imu message as ros standard format
            sensor_msgs::Imu one_imudata;
            // (1) convert the time timestamp from Linux time to GPS time
            double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec * 1e-9;
            timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
            one_imudata.header.stamp = ros::Time(timestamp);
            // (2) store the acce and gyro data
            one_imudata.linear_acceleration.x = imu_msg->acceleration_data.x;
            one_imudata.linear_acceleration.y = imu_msg->acceleration_data.y;
            one_imudata.linear_acceleration.z = imu_msg->acceleration_data.z;
            one_imudata.angular_velocity.x = imu_msg->gyro_data.x;
            one_imudata.angular_velocity.y = imu_msg->gyro_data.y;
            one_imudata.angular_velocity.z = imu_msg->gyro_data.z;

            // store the imu message
            imudatas.push_back(one_imudata);
        }
    }

    // close the file
    bag_in.close();

    return true;
}

/**
 * @brief      extract GNSS solution data from IPS .pos format file and save as RobotGVINS format
 * @note
 *
 * @param[in]   char*      pos_infilepath      filepath to read data
 * @param[out]  list       gnsssol_datas       gnss solutions data of all epochs
 *
 * @return      bool       true      extract successfully
 *                        false      fail to extract

 */
extern bool extract_GNSSSolData_IPSPOSFMT(const char *pos_infilepath, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas)
{
    // 1. Open file to read GNSS solution data
    FILE *infile = fopen(pos_infilepath, "r");
    if (infile == NULL)
    {
        printf("open input data file unsuccessfully!\n");
        return false;
    }

    // 3. Write the GNSS solution data to the file
    char buf[1024] = {'\0'}; // the buffer to store data
    while (!feof(infile))
    {
        // clear old data and read new data
        memset(buf, '\0', sizeof(buf));
        fgets(buf, sizeof(buf), infile);

        // check the length of data
        int charnum = strlen(buf);
        if (charnum <= 0)
            continue;

        // get GNSS solution data defined as .pos format
        // NOTE: XYZCov stores the XY-Var, XZ-Var, and YZ-Var in order
        int GPSWeek = 0, Qfactor = 0, AmbFix = 0;
        double GPSSecond = 0.0, DDOP = 0.0, XYZ[3] = {0.0}, VXYZ[3] = {0.0};
        double XYZVar[3] = {0.0}, XYZCov[3] = {0.0}, VXYZVar[3] = {0.0}, VXYZCov[3] = {0.0};
        sscanf(buf, "%d %lf %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &GPSWeek, &GPSSecond, &Qfactor, &AmbFix, &DDOP,
               &XYZ[0], &XYZ[1], &XYZ[2], &XYZVar[0], &XYZVar[1], &XYZVar[2], &XYZCov[0], &XYZCov[1], &XYZCov[2],
               &VXYZ[0], &VXYZ[1], &VXYZ[2], &VXYZVar[0], &VXYZVar[1], &VXYZVar[2], &VXYZCov[0], &VXYZCov[1], &VXYZCov[2]);

        // create the message
        rosbagio::RobotGVINS_GNSSSol gnsssol_msg;
        double timestamp = GPSWeek * 604800.0 + GPSSecond;
        gnsssol_msg.header.stamp = ros::Time(timestamp);
        gnsssol_msg.Q = Qfactor;
        gnsssol_msg.AmbFix = AmbFix;
        gnsssol_msg.DDOP = DDOP;
        gnsssol_msg.pos_XYZ[0] = XYZ[0], gnsssol_msg.pos_XYZ[1] = XYZ[1], gnsssol_msg.pos_XYZ[2] = XYZ[2];
        gnsssol_msg.vel_XYZ[0] = VXYZ[0], gnsssol_msg.vel_XYZ[1] = VXYZ[1], gnsssol_msg.vel_XYZ[2] = VXYZ[2];
        gnsssol_msg.cov_pos_XYZ[0] = XYZVar[0], gnsssol_msg.cov_pos_XYZ[1] = XYZCov[0], gnsssol_msg.cov_pos_XYZ[2] = XYZCov[1];
        gnsssol_msg.cov_pos_XYZ[3] = XYZCov[0], gnsssol_msg.cov_pos_XYZ[4] = XYZVar[1], gnsssol_msg.cov_pos_XYZ[5] = XYZCov[2];
        gnsssol_msg.cov_pos_XYZ[6] = XYZCov[1], gnsssol_msg.cov_pos_XYZ[7] = XYZCov[2], gnsssol_msg.cov_pos_XYZ[8] = XYZVar[2];
        gnsssol_msg.cov_vel_XYZ[0] = VXYZVar[0], gnsssol_msg.cov_vel_XYZ[1] = VXYZCov[0], gnsssol_msg.cov_vel_XYZ[2] = VXYZCov[1];
        gnsssol_msg.cov_vel_XYZ[3] = VXYZCov[0], gnsssol_msg.cov_vel_XYZ[4] = VXYZVar[1], gnsssol_msg.cov_vel_XYZ[5] = VXYZCov[2];
        gnsssol_msg.cov_vel_XYZ[6] = VXYZCov[1], gnsssol_msg.cov_vel_XYZ[7] = VXYZCov[2], gnsssol_msg.cov_vel_XYZ[8] = VXYZVar[2];

        // store the message
        gnsssol_datas.push_back(gnsssol_msg);
    }

    // close file
    fclose(infile);

    return true;
}

/**
 * @brief       extract GNSS raw data from the bag file and store as IPS observation struct
 * @note        1. Observations data in all epochs will be stored
 *              2. The GNSS raw observations are saved as ublox format (optional)
 *
 * @param[in]   char*                    bag_infilepath           filepath
 * @param[in]   char*                    gnssraw_outfilepath      filepath directory
 * @param[in]   string                   gnssraw_topic            topic
 * @param[out]  list<IPS_OBSDATA>      gnss_obsdata             gnss observations data of all satellites in all epochs
 * @param[out]  list<IPS_GPSEPH>       gnss_ephdata             gnss ephemrtis data of all satellites in all epochs
 *
 * @return      bool       true      extract successfully
 *                        false      fail to extract
 */
extern bool extract_GNSSRawData_VisionRTK2(const char *bag_infilepath, const char *gnssraw_outfilepath, const std::string &gnssraw_topic, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata, std::list<gnss_common::IPS_GPSEPH> &gnss_ephdata)
{
    // 1. Open the bag file to read GNSS raw data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);

    // 2. Open the output file
    // To write GNSS raw data (binary format) to files
    std::ofstream outfile;
    if (gnssraw_outfilepath != NULL)
    {
        outfile.open(gnssraw_outfilepath, std::ios::binary);
        if (!outfile.is_open())
        {
            printf("Fail to open the filepath to write IMU data!\n");
            return false;
        }
    }

    // 3. Prepare variables
    // ros topic
    std::vector<std::string> topics;
    topics.push_back(std::string(gnssraw_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));
    // the variables for rtklib to convert ublox raw data
    raw_t raw;
    init_raw(&raw, STRFMT_UBX); // initilize and assign memory

    // 4. Read data and write to the file
    foreach (rosbag::MessageInstance const m, view)
    {
        // read gnss raw data from the bag file
        if (m.instantiate<rosbagio::VisionRTK2_GNSSRaw>() != nullptr)
        {
            // get the data pointer
            rosbagio::VisionRTK2_GNSSRaw::ConstPtr gnss_msg = m.instantiate<rosbagio::VisionRTK2_GNSSRaw>();

            // initialize the message type
            int message_type = -1;
            // the timestamp to publish ros message
            double timestamp = gnss_msg->header.stamp.sec + gnss_msg->header.stamp.nsec * 1e-9;
            timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;

            // decode the raw data by each char
            for (int i = 0; i < gnss_msg->message.data.size(); i++)
            {
                unsigned char data = gnss_msg->message.data[i];
                message_type = input_raw(&raw, STRFMT_UBX, data);
            }

            // if decode observation, convert to the IPS struct and store
            if (message_type == 1)
            {
                gnss_common::IPS_OBSDATA ips_obsdata;
                Convert_GNSSObsStruct_RTKLIB2IPS(raw.obs.data, raw.obs.n, &ips_obsdata);

                // NOTE: the publish time should be observation time
                ips_obsdata.pubtime = ips_obsdata.gt.GPSWeek * 604800 + ips_obsdata.gt.secsOfWeek + ips_obsdata.gt.fracOfSec;

                gnss_obsdata.push_back(ips_obsdata);
            }

            // if decode ephmeris data, convert to the IPS struct
            if (message_type == 2)
            {
                gnss_common::IPS_GPSEPH ips_eph[IPS_NSATMAX];
                Convert_GNSSNavStruct_RTKLIB2IPS(&raw.nav, ips_eph);

                for (int i = 0; i < IPS_NSATMAX; i++)
                {
                    if (ips_eph[i].toc.GPSWeek <= 0)
                        continue;

                    // NOTE: the publish time should be ROS time
                    ips_eph[i].pubtime = timestamp;

                    gnss_ephdata.push_back(ips_eph[i]);
                }
            }
        }
    }

    // Remember to free the memory
    free_raw(&raw);

    // close the file
    bag_in.close();
    if (gnssraw_outfilepath != NULL)
        outfile.close();

    return true;
}

/**
 * @brief      extract GNSS epoch data from the bag file (Vision-RTK2 data format)
 * @note       The GNSS epoch data are saved as ublox format
 *
 * @param[in]  char*      bag_infilepath            filepath
 * @param[in]  char*      gnssepoch_outfilepath     filepath directory
 * @param[in]  string     gnssepoch_topic           topic
 *
 * @return     bool       true      extract successfully
 *                        false     fail to extract
 */
extern bool extract_GNSSEpochData_VisionRTK2(const char *bag_infilepath, const char *gnssepoch_outfilepath, const std::string &gnssepoch_topic)
{
    // 1. Open the bag file to read GNSS raw data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);

    // 2. Open the output file to write GNSS epoch data
    FILE *outfile = fopen(gnssepoch_outfilepath, "w");
    if (outfile == NULL)
    {
        printf("Fail to open the filepath to write GNSS epoch data!\n");
        return false;
    }

    // 3. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(gnssepoch_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 4. Read data and write to the file
    foreach (rosbag::MessageInstance const m, view)
    {
        if (m.instantiate<rosbagio::VisionRTK2_GNSSEpoch>() != nullptr)
        {
            // gnss epoch data
            rosbagio::VisionRTK2_GNSSEpoch::ConstPtr gnss_epoch = m.instantiate<rosbagio::VisionRTK2_GNSSEpoch>();

            // get the timestamp (if need, convert the Linux time to GPS time)
            long long int timestamp = gnss_epoch->header.stamp.sec * 1e9 + gnss_epoch->header.stamp.nsec;
            long double linuxtime = double(timestamp) * 1e-9;
            long double gpstime = linuxtime - 315964800.0 + 18.0;
            int gpsweek = int(gpstime / 604800.0);
            double gpssecond = gpstime - gpsweek * 604800.0;

            // solution status
            int8_t fixtype = gnss_epoch->fix_type;
            int8_t Qfactor = 6;
            float pdop = gnss_epoch->sol_pdop;

            // position and velocity
            double XYZ[3] = {0.0}, VXYZ[3] = {0.0};
            XYZ[0] = gnss_epoch->pos_X, XYZ[1] = gnss_epoch->pos_Y, XYZ[2] = gnss_epoch->pos_Z;
            VXYZ[0] = gnss_epoch->vel_X, VXYZ[1] = gnss_epoch->vel_Y, VXYZ[2] = gnss_epoch->vel_Z;

            // the covariance of position and velocity
            boost::array<double, 9> XYZCov, VXYZCov;
            XYZCov = gnss_epoch->cov_pos_xyz;
            VXYZCov = gnss_epoch->cov_vel_xyz;

            fprintf(outfile, "%6d %12.6f %3d %3d %6.2f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f %15.9f\n",
                    gpsweek, gpssecond, Qfactor, fixtype, pdop, XYZ[0], XYZ[1], XYZ[2], XYZCov[0], XYZCov[4], XYZCov[1], XYZCov[2], XYZCov[5], XYZCov[8], VXYZ[0], VXYZ[1], VXYZ[2], VXYZCov[0], VXYZCov[4], VXYZCov[1], VXYZCov[2], VXYZCov[5], VXYZCov[8]);
        }
    }

    bag_in.close();
    fclose(outfile);

    return true;
}

/**
 * @brief      extract GNSS solution data from the bag file (HKUST-Aerial-GVINS format) and save as RobotGVINS format
 * @note
 *
 * @param[in]  char*       bag_infilepath      bag file
 * @param[in]  string      gnsssol_topic       gnss solution topic
 * @param[out] list        gnsssol_datas       gnss solution data
 *
 * @return     bool       true      extract successfully
 *                        false     fail to extract
 */
extern bool extract_GNSSSolData_HKUSTGVINS(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas)
{

    // 1. Open the bag file to read GNSS solution data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);
    if (!bag_in.isOpen())
    {
        printf("open ros bag file to extract data unsuccessfully!\n");
        return false;
    }

    // 2. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(gnsssol_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 3. Extract each meassage
    foreach (rosbag::MessageInstance const m, view)
    {
        if (m.instantiate<rosbagio::GVINS_GNSSPVTSolnMsg>() != nullptr)
        {
            // get gnss solution data
            rosbagio::GVINS_GNSSPVTSolnMsg::ConstPtr gnsssol_msg = m.instantiate<rosbagio::GVINS_GNSSPVTSolnMsg>();

            // convert the HKUST-GVINS format to RobotGVINS format
            double timestamp = 0.0, LLH[3] = {0.0}, VENU[3] = {0.0}, XYZ[3] = {0.0}, VXYZ[3] = {0.0};
            double XYZVar[3] = {0.0}, XYZCov[3] = {0.0}, VXYZVar[3] = {0.0}, VXYZCov[3] = {0.0};
            // (1) convert position from LLH to ECEF
            LLH[0] = gnsssol_msg->latitude * IPS_D2R, LLH[1] = gnsssol_msg->longitude * IPS_D2R, LLH[2] = gnsssol_msg->altitude;
            gnss_common::LLH2XYZ(LLH, XYZ);
            // (2) convert velocity from ENU to ECEF
            VENU[0] = gnsssol_msg->vel_e, VENU[1] = gnsssol_msg->vel_n, VENU[2] = -gnsssol_msg->vel_d;
            Eigen::Matrix3d R_nToe = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
            Eigen::Vector3d VENU_vec = Eigen::Vector3d::Zero();
            VENU_vec << VENU[0], VENU[1], VENU[2];
            Eigen::Vector3d VXYZ_vec = R_nToe * VENU_vec;
            VXYZ[0] = VXYZ_vec[0], VXYZ[1] = VXYZ_vec[1], VXYZ[2] = VXYZ_vec[2];
            // (3) compute the position and velocity variance
            XYZVar[0] = XYZVar[1] = XYZVar[2] = sqrt(pow(gnsssol_msg->h_acc, 2) + pow(gnsssol_msg->v_acc, 2));
            VXYZVar[0] = VXYZVar[1] = VXYZVar[2] = gnsssol_msg->vel_acc;

            // store the solution data
            rosbagio::RobotGVINS_GNSSSol sol_data;
            timestamp = gnsssol_msg->time.week * 604800.0 + gnsssol_msg->time.tow;
            sol_data.header.stamp = ros::Time(timestamp);
            sol_data.Q = 1;
            sol_data.AmbFix = 5;
            sol_data.DDOP = gnsssol_msg->p_dop;
            sol_data.pos_XYZ[0] = XYZ[0], sol_data.pos_XYZ[1] = XYZ[1], sol_data.pos_XYZ[2] = XYZ[2];
            sol_data.vel_XYZ[0] = VXYZ[0], sol_data.vel_XYZ[1] = VXYZ[1], sol_data.vel_XYZ[2] = VXYZ[2];
            sol_data.cov_pos_XYZ[0] = XYZVar[0], sol_data.cov_pos_XYZ[1] = XYZCov[0], sol_data.cov_pos_XYZ[2] = XYZCov[1];
            sol_data.cov_pos_XYZ[3] = XYZCov[0], sol_data.cov_pos_XYZ[4] = XYZVar[1], sol_data.cov_pos_XYZ[5] = XYZCov[2];
            sol_data.cov_pos_XYZ[6] = XYZCov[1], sol_data.cov_pos_XYZ[7] = XYZCov[2], sol_data.cov_pos_XYZ[8] = XYZVar[2];
            sol_data.cov_vel_XYZ[0] = VXYZVar[0], sol_data.cov_vel_XYZ[1] = VXYZCov[0], sol_data.cov_vel_XYZ[2] = VXYZCov[1];
            sol_data.cov_vel_XYZ[3] = VXYZCov[0], sol_data.cov_vel_XYZ[4] = VXYZVar[1], sol_data.cov_vel_XYZ[5] = VXYZCov[2];
            sol_data.cov_vel_XYZ[6] = VXYZCov[1], sol_data.cov_vel_XYZ[7] = VXYZCov[2], sol_data.cov_vel_XYZ[8] = VXYZVar[2];

            gnsssol_datas.push_back(sol_data);
        }
    }

    bag_in.close();

    return true;
}

/**
 * @brief      extract GNSS solution data from the bag file (KAIST vrs_gps format) and save as RobotGVINS format
 * @note       the gnss solution of ros format has no velocity information
 *
 * @param[in]  char*       bag_infilepath      bag file
 * @param[in]  string      gnsssol_topic       gnss solution topic
 * @param[out] list        gnsssol_datas       gnss solution data
 *
 * @return     bool       true      extract successfully
 *                        false     fail to extract
 */
extern bool extract_GNSSSolData_KAIST_VRSGPS(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas)
{
    // 1. Open the bag file to read GNSS solution data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);
    if (!bag_in.isOpen())
    {
        printf("open ros bag file to extract data unsuccessfully!\n");
        return false;
    }

    // 2. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(gnsssol_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 3. Extract each meassage
    foreach (rosbag::MessageInstance const m, view)
    {
        if (m.instantiate<rosbagio::KAIST_VRSGPS>() != nullptr)
        {
            // get gnss solution data
            rosbagio::KAIST_VRSGPS::ConstPtr gnsssol_msg = m.instantiate<rosbagio::KAIST_VRSGPS>();

            // convert the KAIST format to RobotGVINS format
            // (1) convert position from LLH to ECEF
            double LLH[3] = {0.0}, XYZ[3] = {0.0};
            LLH[0] = gnsssol_msg->latitude * IPS_D2R, LLH[1] = gnsssol_msg->longitude * IPS_D2R, LLH[2] = gnsssol_msg->altitude;
            gnss_common::LLH2XYZ(LLH, XYZ);
            // (2) get the position covariance in ENU
            Eigen::Matrix3d ENUCov = Eigen::Matrix3d::Zero();
            ENUCov(0, 0) = pow(gnsssol_msg->lat_std, 2), ENUCov(0, 1) = 0.0, ENUCov(0, 2) = 0.0;
            ENUCov(1, 0) = 0.0, ENUCov(1, 1) = pow(gnsssol_msg->lon_std, 2), ENUCov(1, 2) = 0.0;
            ENUCov(2, 0) = 0.0, ENUCov(2, 1) = 0.0, ENUCov(2, 2) = pow(gnsssol_msg->altitude_std, 2);
            // (3) convert position covariance from ENU to ECEF
            Eigen::Matrix3d R_nToe = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
            Eigen::Matrix3d XYZCov = R_nToe * ENUCov * R_nToe.transpose();

            // store the gnss solution data
            rosbagio::RobotGVINS_GNSSSol one_data;
            double timestamp = gnsssol_msg->header.stamp.sec + gnsssol_msg->header.stamp.nsec * 1e-9;
            timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
            one_data.header.stamp = ros::Time(timestamp);
            one_data.header.frame_id = gnsssol_msg->header.frame_id;
            one_data.AmbFix = 5;
            one_data.DDOP = 1.5;
            one_data.pos_XYZ[0] = XYZ[0], one_data.pos_XYZ[1] = XYZ[1], one_data.pos_XYZ[2] = XYZ[2];
            one_data.vel_XYZ[0] = 0.0, one_data.vel_XYZ[1] = 0.0, one_data.vel_XYZ[2] = 0.0;
            one_data.cov_pos_XYZ[0] = XYZCov(0, 0), one_data.cov_pos_XYZ[1] = XYZCov(0, 1), one_data.cov_pos_XYZ[2] = XYZCov(0, 2);
            one_data.cov_pos_XYZ[3] = XYZCov(1, 0), one_data.cov_pos_XYZ[4] = XYZCov(1, 1), one_data.cov_pos_XYZ[5] = XYZCov(1, 2);
            one_data.cov_pos_XYZ[6] = XYZCov(2, 0), one_data.cov_pos_XYZ[7] = XYZCov(2, 1), one_data.cov_pos_XYZ[8] = XYZCov(2, 2);
            one_data.cov_vel_XYZ[0] = 0.0, one_data.cov_vel_XYZ[1] = 0.0, one_data.cov_vel_XYZ[2] = 0.0;
            one_data.cov_vel_XYZ[3] = 0.0, one_data.cov_vel_XYZ[4] = 0.0, one_data.cov_vel_XYZ[5] = 0.0;
            one_data.cov_vel_XYZ[6] = 0.0, one_data.cov_vel_XYZ[7] = 0.0, one_data.cov_vel_XYZ[8] = 0.0;

            gnsssol_datas.push_back(one_data);
        }
    }

    // close the file
    bag_in.close();

    return true;
}

/**
 * @brief      extract GNSS solution data from the bag file (ros standard format) and save as RobotGVINS format
 * @note       the gnss solution of ros format has no velocity information
 *
 * @param[in]  char*       bag_infilepath      bag file
 * @param[in]  string      gnsssol_topic       gnss solution topic
 * @param[out] list        gnsssol_datas       gnss solution data
 *
 * @return     bool       true      extract successfully
 *                        false     fail to extract
 */
extern bool extract_GNSSSolData_ROSFormat(const char *bag_infilepath, const std::string &gnsssol_topic, std::list<rosbagio::RobotGVINS_GNSSSol> &gnsssol_datas)
{

    // 1. Open the bag file to read GNSS solution data
    rosbag::Bag bag_in;
    bag_in.open(bag_infilepath, rosbag::bagmode::Read);
    if (!bag_in.isOpen())
    {
        printf("open ros bag file to extract data unsuccessfully!\n");
        return false;
    }

    // 2. Prepare variables
    std::vector<std::string> topics;
    topics.push_back(std::string(gnsssol_topic));
    rosbag::View view(bag_in, rosbag::TopicQuery(topics));

    // 3. Extract each meassage
    foreach (rosbag::MessageInstance const m, view)
    {
        if (m.instantiate<sensor_msgs::NavSatFix>() != nullptr)
        {
            // get gnss solution data
            sensor_msgs::NavSatFix::ConstPtr gnsssol_msg = m.instantiate<sensor_msgs::NavSatFix>();

            // convert the ros standard format to RobotGVINS format
            // (1) convert position from LLH to ECEF
            double LLH[3] = {0.0}, XYZ[3] = {0.0};
            LLH[0] = gnsssol_msg->latitude * IPS_D2R, LLH[1] = gnsssol_msg->longitude * IPS_D2R, LLH[2] = gnsssol_msg->altitude;
            gnss_common::LLH2XYZ(LLH, XYZ);
            // (2) get the position covariance in ENU
            Eigen::Matrix3d ENUCov = Eigen::Matrix3d::Zero();
            ENUCov(0, 0) = gnsssol_msg->position_covariance[0], ENUCov(0, 1) = gnsssol_msg->position_covariance[1], ENUCov(0, 2) = gnsssol_msg->position_covariance[2];
            ENUCov(1, 0) = gnsssol_msg->position_covariance[3], ENUCov(1, 1) = gnsssol_msg->position_covariance[4], ENUCov(1, 2) = gnsssol_msg->position_covariance[5];
            ENUCov(2, 0) = gnsssol_msg->position_covariance[6], ENUCov(2, 1) = gnsssol_msg->position_covariance[7], ENUCov(2, 2) = gnsssol_msg->position_covariance[8];
            // (3) convert position covariance from ENU to ECEF
            Eigen::Matrix3d R_nToe = gnss_common::ComputeRotMat_ENU2ECEF(LLH[0], LLH[1]);
            Eigen::Matrix3d XYZCov = R_nToe * ENUCov * R_nToe.transpose();

            // store the gnss solution data
            rosbagio::RobotGVINS_GNSSSol one_data;
            double timestamp = gnsssol_msg->header.stamp.sec + gnsssol_msg->header.stamp.nsec * 1e-9;
            timestamp = timestamp - GPS_LINUX_TIME + LEAP_SECOND;
            one_data.header.stamp = ros::Time(timestamp);
            one_data.header.frame_id = gnsssol_msg->header.frame_id;
            one_data.AmbFix = 5;
            one_data.DDOP = 1.5;
            one_data.pos_XYZ[0] = XYZ[0], one_data.pos_XYZ[1] = XYZ[1], one_data.pos_XYZ[2] = XYZ[2];
            one_data.vel_XYZ[0] = 0.0, one_data.vel_XYZ[1] = 0.0, one_data.vel_XYZ[2] = 0.0;
            one_data.cov_pos_XYZ[0] = XYZCov(0, 0), one_data.cov_pos_XYZ[1] = XYZCov(0, 1), one_data.cov_pos_XYZ[2] = XYZCov(0, 2);
            one_data.cov_pos_XYZ[3] = XYZCov(1, 0), one_data.cov_pos_XYZ[4] = XYZCov(1, 1), one_data.cov_pos_XYZ[5] = XYZCov(1, 2);
            one_data.cov_pos_XYZ[6] = XYZCov(2, 0), one_data.cov_pos_XYZ[7] = XYZCov(2, 1), one_data.cov_pos_XYZ[8] = XYZCov(2, 2);
            one_data.cov_vel_XYZ[0] = 0.0, one_data.cov_vel_XYZ[1] = 0.0, one_data.cov_vel_XYZ[2] = 0.0;
            one_data.cov_vel_XYZ[3] = 0.0, one_data.cov_vel_XYZ[4] = 0.0, one_data.cov_vel_XYZ[5] = 0.0;
            one_data.cov_vel_XYZ[6] = 0.0, one_data.cov_vel_XYZ[7] = 0.0, one_data.cov_vel_XYZ[8] = 0.0;

            gnsssol_datas.push_back(one_data);
        }
    }

    return true;
}

/**
 * @brief      Extract GNSS observation data from the rinex 3.0x file (IPS version)
 * @note       The GNSS obs data are saved as IPS struct format
 *
 * @param[in]  char*     rinex_infilepath      rinex format file
 * @param[out] list      gnss_obsdata          store all observations data in all epochs
 *
 * @return     bool       true      extract successfully
 *                        false     fail to extract
 */
extern bool extract_GNSSObsData_RINEX3_IPSVersion(const char *rinex_infilepath, std::list<gnss_common::IPS_OBSDATA> &gnss_obsdata)
{
    // 1. Open the rinex file
    FILE *ifp = fopen(rinex_infilepath, "rt");
    if (!ifp)
    {
        printf("Fail to open rinex observation file!\n");
        return false;
    }

    // 2. Prepare variables
    double dVal = 0;
    const char sys_str[IPS_NSYS + 1] = {"GRCCEJ"};
    char buff[IPS_MAXSIZE], *label = buff + 60, ch[128] = {'\0'};
    int version = 0, prn = 0, sys = 0, sys_id = 0, GNSSTypeNum[IPS_NSYS] = {0};
    std::string timeSys = "GPS";
    std::vector<std::string> GNSSType[IPS_NSYS];
    std::map<std::string, double> PhaseShift[IPS_NSYS];
    gnss_common::IPS_OBSHEAD obsHead;
    int GNSSObsPos[IPS_NSYS][4 * NFREQ] = {{0}}; // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
    int GPSObsPosX[4 * NFREQ] = {0};             // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
    std::string CodeType[4] = {"C", "L", "D", "S"};
    std::string GNSSCodePris[IPS_NSYS][5] = {
        {"CPYWMNSLX", "PYWCMNDSLX", "IQX", "", ""}, // GPS
        {"PC", "PC", "IQX", "", ""},                // GLO
        {"IQX", "IQX", "IQXA", "", ""},             // BD2
        {"IQX", "IQX", "IQXA", "DPXA", "DPX"},      // BD3
        {"CABXZ", "IQX", "IQX", "IQX", "ABCXZ"},    // GAL
        {"CSLXZ", "SLX", "IQXDPZ", "SLXEZ", ""},    // QZS
    };
    std::string GNSSCodeFreq[IPS_NSYS][5] = {
        {"1", "2", "5", " ", " "}, // GPS
        {"1", "2", "3", " ", " "}, // GLO
        {"2", "7", "6", " ", " "}, // BD2
        {"2", "7", "6", "1", "5"}, // BD3
        {"1", "5", "7", "8", "6"}, // GAL
        {"1", "2", "5", "6", " "}, // QZS
    };
    std::vector<std::string> GNSSTypeRead[IPS_NSYS];
    for (int i = 0; i < IPS_NSYS; i++)
    {
        GNSSTypeRead[i].resize(4 * NFREQ, "   ");
    }

    // switch the gnss frequency
    if (gnss_common::gs_bSwitchGNSSFrq)
    {
        for (int f = 0; f < NFREQ; f++)
        {
            if (gnss_common::gs_strBD2Frq[f] == "B1I")
            {
                GNSSCodePris[IPS_ISYSBD2][f] = "IQX";
                GNSSCodeFreq[IPS_ISYSBD2][f] = "2";
            }
            else if (gnss_common::gs_strBD2Frq[f] == "B2I")
            {
                GNSSCodePris[IPS_ISYSBD2][f] = "IQX";
                GNSSCodeFreq[IPS_ISYSBD2][f] = "7";
            }
            else if (gnss_common::gs_strBD2Frq[f] == "B3I")
            {
                GNSSCodePris[IPS_ISYSBD2][f] = "IQXA";
                GNSSCodeFreq[IPS_ISYSBD2][f] = "6";
            }
        }

        for (int f = 0; f < NFREQ; f++)
        {
            if (gnss_common::gs_strBD3Frq[f] == "B1I")
            {
                GNSSCodePris[IPS_ISYSBD3][f] = "IQX";
                GNSSCodeFreq[IPS_ISYSBD3][f] = "2";
            }
            else if (gnss_common::gs_strBD3Frq[f] == "B2I" || gnss_common::gs_strBD3Frq[f] == "B2b")
            {
                GNSSCodePris[IPS_ISYSBD3][f] = "IQX";
                GNSSCodeFreq[IPS_ISYSBD3][f] = "7";
            }
            else if (gnss_common::gs_strBD3Frq[f] == "B3I")
            {
                GNSSCodePris[IPS_ISYSBD3][f] = "IQXA";
                GNSSCodeFreq[IPS_ISYSBD3][f] = "6";
            }
            else if (gnss_common::gs_strBD3Frq[f] == "B1C")
            {
                GNSSCodePris[IPS_ISYSBD3][f] = "DPXA";
                GNSSCodeFreq[IPS_ISYSBD3][f] = "1";
            }
            else if (gnss_common::gs_strBD3Frq[f] == "B2a")
            {
                GNSSCodePris[IPS_ISYSBD3][f] = "DPX";
                GNSSCodeFreq[IPS_ISYSBD3][f] = "5";
            }
        }
    }

    // 3. Read the header info
    while (fgets(buff, IPS_MAXSIZE, ifp))
    {
        if (strstr(label, "RINEX VERSION / TYPE"))
        {
            xstrmid(buff, 5, 1, ch);
            version = atoi(ch);
        }
        else if (strstr(label, "REC # / TYPE / VERS"))
        {
            xstrmid(buff, 20, 20, obsHead.recType);
        }
        else if (strstr(label, "ANT # / TYPE"))
        {
            xstrmid(buff, 20, 20, obsHead.antType);
        }
        else if (strstr(label, "APPROX POSITION XYZ"))
        {
            obsHead.XYZ[0] = str2num(buff, 0, 14);
            obsHead.XYZ[1] = str2num(buff, 14, 14);
            obsHead.XYZ[2] = str2num(buff, 28, 14);
        }
        else if (strstr(label, "ANTENNA: DELTA H/E/N"))
        {
            obsHead.ant[2] = str2num(buff, 0, 14);
            obsHead.ant[0] = str2num(buff, 14, 14);
            obsHead.ant[1] = str2num(buff, 28, 14);
        }
        else if (strstr(label, "SYS / # / OBS TYPES"))
        {
            sys_id = -1;

            for (int k = 0; k < IPS_NSYS; k++)
            {
                if (buff[0] == sys_str[k])
                {
                    sys_id = k;
                    break;
                }
            }

            if (sys_id < 0)
                continue;

            GNSSTypeNum[sys_id] = (int)str2num(buff, 3, 3);

            for (int i = 0, j = 7; i < GNSSTypeNum[sys_id]; i++, j += 4)
            {
                if (j > 58)
                {
                    if (!fgets(buff, IPS_MAXSIZE, ifp))
                        return false;
                    j = 7;
                }

                xstrmid(buff, j, 3, ch);

                if (buff[0] == 'C' && (ch[2] == 'I' || ch[2] == 'Q') && ch[1] == '1')
                    ch[1] = '2';

                GNSSType[sys_id].push_back(std::string(ch));
            }

            if (sys_id == IPS_ISYSBD2)
            {
                GNSSTypeNum[IPS_ISYSBD3] = GNSSTypeNum[IPS_ISYSBD2];
                GNSSType[IPS_ISYSBD3] = GNSSType[IPS_ISYSBD2];
            }
        }
        else if (strstr(label, "SYS / PHASE SHIFT"))
        {
            sys_id = -1;

            for (int k = 0; k < IPS_NSYS; k++)
            {
                if (buff[0] == sys_str[k])
                {
                    sys_id = k;
                    break;
                }
            }

            if (sys_id < 0)
                continue;

            xstrmid(buff, 2, 3, ch);
            dVal = str2num(buff, 6, 8);
            PhaseShift[sys_id][std::string(ch)] = dVal;

            if (sys_id == IPS_ISYSBD2)
            {
                PhaseShift[IPS_ISYSBD3][std::string(ch)] = dVal;
            }
        }
        else if (strstr(label, "INTERVAL"))
        {
            obsHead.dt = str2num(buff, 0, 60);
        }
        else if (strstr(label, "TIME OF FIRST OBS"))
        {
            xstrmid(buff, 48, 3, ch);
            if (ch[0] != ' ')
                timeSys = std::string(ch);
        }
        else if (strstr(label, "END OF HEADER"))
        {
            break;
        }
    }

    if (version != 3)
    {
        printf("RINEX VERSION is not 3.0!\n");
        return false;
    }

    if (timeSys != std::string("GPS"))
    {
        printf("Time system is not GPS!\n");
        return false;
    }

    bool bflag = false;

    for (sys_id = 0; sys_id < IPS_NSYS; sys_id++)
    {
        for (int ncode = 0; ncode < 4; ncode++)
        {
            for (int frq = 0; frq < NFREQ; frq++)
            {
                bflag = false;

                if (sys_id == 0)
                {
                    std::string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + "X";
                    for (int j = 0; j < (int)GNSSType[sys_id].size(); j++)
                    {
                        if (code == GNSSType[sys_id][j])
                        {
                            GPSObsPosX[ncode * NFREQ + frq] = j + 1;
                            break;
                        }
                    }
                }

                for (int i = 0; i < (int)GNSSCodePris[sys_id][frq].size(); i++)
                {
                    std::string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + GNSSCodePris[sys_id][frq][i];

                    for (int j = 0; j < (int)GNSSType[sys_id].size(); j++)
                    {
                        if (code == GNSSType[sys_id][j])
                        {
                            GNSSObsPos[sys_id][ncode * NFREQ + frq] = j + 1;
                            GNSSTypeRead[sys_id][ncode * NFREQ + frq] = code;
                            bflag = true;
                            break;
                        }
                    }

                    if (bflag)
                        break;
                }
            }
        }
    }

    double PhaseShiftCorr[IPS_NSYS][NFREQ] = {0.0};
    double PhaseShiftCorrX[NFREQ] = {0.0};
    for (int i = 0; i < IPS_NSYS; i++)
    {
        for (int j = 0; j < NFREQ; j++)
        {
            PhaseShiftCorr[i][j] = PhaseShift[i][GNSSTypeRead[i][NFREQ + j]];
        }
    }
    PhaseShiftCorrX[0] = -PhaseShift[0]["L1X"];
    PhaseShiftCorrX[1] = -PhaseShift[0]["L2X"];
    PhaseShiftCorrX[2] = -PhaseShift[0]["L5X"];

    /// 4. Read the body info
    int SatSYS = IPS_SYSALL;
    int flag = 0; // event flag
    int nsat = 0; // satellite number

    // read the observation data in each epoch
    while (fgets(buff, IPS_MAXSIZE, ifp))
    {
        if (feof(ifp))
            break;

        gnss_common::IPS_OBSDATA obsData;

        /* decode obs epoch */
        {
            if (buff[0] != '>')
                continue;

            /* epoch flag: 3:new site,4:header info,5:external event */
            nsat = (int)str2num(buff, 32, 3);
            if (nsat <= 0)
                continue;

            flag = (int)str2num(buff, 31, 1);
            if (3 <= flag && flag <= 5)
            {
                // 3-5 represents the time info
                for (int p = 0; p < nsat; p++)
                    fgets(buff, IPS_MAXSIZE, ifp);
                continue;
            }

            obsData.gt = gnss_common::str2time(buff, 1, 28);

            int nsatValid = 0; // the number of available satellites

            for (int i = 0; i < nsat; i++)
            {
                gnss_common::IPS_OBSDATA_t obst;

                fgets(buff, IPS_MAXSIZE, ifp);

                xstrmid(buff, 0, 3, ch);
                prn = gnss_common::satno2prn(ch);
                if (prn == 0)
                    continue;

                sys = IPS_SYSNON;
                gnss_common::satprn2no(prn, &sys);

                bool bpush = false;

                sys_id = gnss_common::Sys2Index(sys);

                if (SatSYS & sys)
                {
                    bpush = true;
                    int pos = 0;
                    for (int k = 0; k < NFREQ; k++)
                    {
                        bool bX = false;

                        // L
                        pos = GNSSObsPos[sys_id][NFREQ + k];
                        if (pos > 0)
                        {
                            pos--;
                            obst.L[k] = str2num(buff, 3 + 16 * pos, 14);
                            obst.LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
                            obst.SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
                            if (obst.L[k] != 0.0)
                                obst.L[k] += PhaseShiftCorr[0][k];

                            bX = (sys == IPS_SYSGPS && obst.L[k] == 0.0 && GNSSTypeRead[sys_id][NFREQ + k][2] == 'W');
                        }

                        if (bX)
                        {
                            pos = GPSObsPosX[NFREQ + k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.L[k] = str2num(buff, 3 + 16 * pos, 14);
                                obst.LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
                                obst.SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
                                if (obst.L[k] != 0.0)
                                    obst.L[k] += PhaseShiftCorrX[k];
                            }

                            // P
                            pos = GPSObsPosX[k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.P[k] = str2num(buff, 3 + 16 * pos, 14);
                            }
                            obst.code[k][0] = GNSSTypeRead[sys_id][k][1];
                            obst.code[k][1] = 'X';

                            // S
                            pos = GPSObsPosX[NFREQ * 3 + k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
                            }
                        }

                        if (obst.P[k] == 0.0)
                        {
                            pos = GNSSObsPos[sys_id][k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.P[k] = str2num(buff, 3 + 16 * pos, 14);
                            }
                            obst.code[k][0] = GNSSTypeRead[sys_id][k][1];
                            obst.code[k][1] = GNSSTypeRead[sys_id][k][2];
                        }

                        // S
                        if (obst.S[k] == 0.0)
                        {
                            pos = GNSSObsPos[sys_id][NFREQ * 3 + k];
                            if (pos > 0)
                            {
                                pos--;
                                obst.S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
                            }
                        }

                        // D
                        pos = GNSSObsPos[sys_id][NFREQ * 2 + k];
                        if (pos > 0)
                        {
                            pos--;
                            obst.D[k] = str2num(buff, 3 + 16 * pos, 14);
                        }
                    }
                }

                if (bpush)
                {
                    obst.prn = prn;
                    obsData.obs.push_back(obst);
                    nsatValid++;
                }
            }

            obsData.nsat = nsatValid;
            obsData.flag = flag;
        }

        SortGNSSObs_IPSStruct(&obsData);
        obsData.pubtime = obsData.gt.GPSWeek * 604800 + obsData.gt.secsOfWeek + obsData.gt.fracOfSec;
        gnss_obsdata.push_back(obsData);
    }

    return true;
}