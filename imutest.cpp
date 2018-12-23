// Sorry for this one...
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// Includes
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <thread>

// Define what PCL types we are using...
typedef pcl::PointXYZ WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

WSPointCloudPtr points_to_pcl_no_texture(const rs2::points &points)
{

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    WSPointCloudPtr cloud(new WSPointCloud());

    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto vertices = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;
    }

    return cloud;
}

class GyroBias
{
  private:
    int calibrationUpdates;
    double minX, maxX;
    double minY, maxY;
    double minZ, maxZ;


  public:
    bool isSet;

    double x;
    double y;
    double z;

    GyroBias()
    {
        reset();
    }

    void reset()
    {
        calibrationUpdates = 0;

        minX = 1000;
        minY = 1000;
        minZ = 1000;
        maxX = -1000;
        maxY = -1000;
        maxZ = -1000;

        x = 0;
        y = 0;
        z = 0;

        isSet = false;
    }


    bool update(double gx, double gy, double gz)
    {
        if (calibrationUpdates < 50)
        {   
            maxX = std::max(gx, maxX);
            maxY = std::max(gy, maxY);
            maxZ = std::max(gz, maxZ);

            minX = std::min(gx, minX);
            minY = std::min(gy, minY);
            minZ = std::min(gz, minZ);

            calibrationUpdates++;
            return false;
        }
        else if (calibrationUpdates == 50)
        {
            x = (maxX + minX)/2.0;
            y = (maxY + minY)/2.0;
            z = (maxZ + minZ)/2.0;
            calibrationUpdates++;

            /*std::cout << "BIAS-X: " << minX << " - " << maxX << std::endl;
            std::cout << "BIAS-Y: " << minY << " - " << maxY << std::endl;
            std::cout << "BIAS-Z: " << minZ << " - " << maxZ << std::endl;
            std::cout << "BIAS: " << x << " " << y << " " << z << std::endl;
            */
            isSet = true;

            return true;
        }
        else
        {
            return false;
        }
    }
};

int main()
{

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);

    //
    // Create the PCL visualizer
    //
    pcl::visualization::PCLVisualizer viewer("whitestick");

    // Add some features to our viewport
    viewer.addCoordinateSystem(1.0, "axis", 0);

    float thetaX = 0.0;
    float thetaY = 0.0;
    float thetaZ = 0.0;

    GyroBias bias;

    //
    // Start capturing
    //
    pipe.start(cfg);

    bool firstAccel = true;
    double last_ts[RS2_STREAM_COUNT];
    double dt[RS2_STREAM_COUNT];

    viewer.setCameraPosition(10.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0);

    // Main loop
    while (!viewer.wasStopped())
    {
        WSPointCloudPtr cloud = NULL;

        rs2::frameset frames;
        if (!pipe.poll_for_frames(&frames))
        {
            // Redraw etc
            viewer.spinOnce();
            continue;
        }

        //std::cout << "Num frames: " << frames.size() << " ";
        for (auto f : frames)
        {
            rs2::stream_profile profile = f.get_profile();

            unsigned long fnum = f.get_frame_number();
            double ts = f.get_timestamp();
            dt[profile.stream_type()] = (ts - last_ts[profile.stream_type()] ) / 1000.0;
            last_ts[profile.stream_type()] = ts;

            //std::cout << "[ " << profile.stream_name() << " fnum: " << fnum << " dt: " << dt[profile.stream_type()] << "] ";
        }
        //std::cout << std::endl;

        auto depth = frames.get_depth_frame();
        auto colored_frame = frames.get_color_frame();

        auto fa = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        rs2::motion_frame accel = fa.as<rs2::motion_frame>();

        auto fg = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        rs2::motion_frame gyro = fg.as<rs2::motion_frame>();

        if (gyro)
        {
            rs2_vector gv = gyro.get_motion_data();

            //bias.update(gv.x, gv.y, gv.z);

            double gyroX = gv.x - bias.x;
            double gyroY = gv.y - bias.y;
            double gyroZ = gv.z - bias.z;

            gyroX *= dt[RS2_STREAM_GYRO];
            gyroY *= dt[RS2_STREAM_GYRO];
            gyroZ *= dt[RS2_STREAM_GYRO];

            //std::cout << "dt=" << dt[RS2_STREAM_GYRO] << " gyroX=" << gyroX * 180.0 / M_PI << " gyroY=" << gyroY * 180.0 / M_PI << " gyroZ=" << gyroZ * 180.0 / M_PI << std::endl;
            //std::cout << " gyroX=" << gyroX * 180.0 / M_PI << " gyroY=" << gyroY * 180.0 / M_PI << " gyroZ=" << gyroZ * 180.0 / M_PI << std::endl;
            //std::cout << last_ts[RS2_STREAM_GYRO] / 1000.0 << " " << gyroZ * 180.0 / M_PI << std::endl;

            // 
            // Get compensation from GYRO
            // 

            // Around "blue", poisitive => right
            thetaX += gyroZ;

            // Yaw...don't compensate...
            thetaY += 0.0;

            // Around "red", positve => right
            thetaZ -= gyroX; 

        }

        if (accel)
        {
            rs2_vector av = accel.get_motion_data();
            float R = sqrtf(av.x * av.x + av.y * av.y + av.z * av.z);

            float accelX = acos(av.x / R);
            float accelY = acos(av.y / R);
            float accelZ = acos(av.z / R);

            //std::cout << "accelX=" << accelX*180.0/M_PI << " accelY=" << accelY*180.0/M_PI << " accelZ=" << accelZ*180.0/M_PI << std::endl;

            if (firstAccel)
            {
                firstAccel = false;
                thetaX = accelX;
                thetaY = accelY;
                thetaZ = accelZ;
            }
            else
            {
                // Compensate GYRO-drift with ACCEL
                thetaX = thetaX * 0.98 + accelX * 0.02;
                thetaY = thetaY * 0.98 + accelY * 0.02;
                thetaZ = thetaZ * 0.98 + accelZ * 0.02;
            }

            //std::cout << "thetaX=" << thetaX*180.0/M_PI << " thetaY=" << thetaY*180.0/M_PI << " thetaZ=" << thetaZ*180.0/M_PI << std::endl;
            //std::cout << last_ts[RS2_STREAM_GYRO] / 1000.0 << " " << thetaX*180.0/M_PI << " " << thetaY*180.0/M_PI << " " << thetaZ*180.0/M_PI << std::endl;
            //std::cout << last_ts[RS2_STREAM_GYRO] / 1000.0 << " " << thetaX*180.0/M_PI << std::endl;

        }

        auto points = pc.calculate(depth);

        // Actual calling of conversion and saving XYZRGB cloud to file
        cloud = points_to_pcl_no_texture(points);

        // Downsize to a voxel grid (i.e small boxes of the cloud)
        WSPointCloudPtr voxel_cloud(new WSPointCloud());
        pcl::VoxelGrid<WSPoint> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.07f, 0.07f, 0.07f);
        sor.filter(*voxel_cloud);

        // Transform the cloud according to built in IMU (to get it straight)
        Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(-(thetaZ - M_PI_2), Eigen::Vector3f(1, 0, 0)));
        Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(0.0, Eigen::Vector3f(0, 1, 0)));
        Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(thetaX - M_PI_2, Eigen::Vector3f(0, 0, 1)));
        Eigen::Affine3f rot = rz * ry * rx;

        // Executing the transformation
        pcl::PointCloud<WSPoint>::Ptr transformed_cloud(new WSPointCloud());
        pcl::transformPointCloud(*voxel_cloud, *transformed_cloud, rot);

        // Show camera as coordinate system
        viewer.removeCoordinateSystem("camera");
        viewer.addCoordinateSystem(0.5, rot, "camera", 0);

        if (!viewer.updatePointCloud(transformed_cloud, "cloud"))
        {
            // Add to the viewer
            viewer.addPointCloud<WSPoint>(transformed_cloud, "cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
        }

        // Redraw etc
        viewer.spinOnce();
    }

    return 0;
}
