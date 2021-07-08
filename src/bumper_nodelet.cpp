/* headers //{ */

// clang: MatousFormat
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <boost/circular_buffer.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/cyclic.h>
/* #include <mrs_lib/vector_converter.h> */

#include <mrs_bumper/BumperConfig.h>
#include <mrs_msgs/ObstacleSectors.h>
#include <mrs_msgs/Histogram.h>

// shortcut type to the dynamic reconfigure manager template instance
typedef mrs_lib::DynamicReconfigureMgr<mrs_bumper::BumperConfig> drmgr_t;

//}

namespace mrs_bumper
{

  using ObstacleSectors = mrs_msgs::ObstacleSectors;
  using Histogram = mrs_msgs::Histogram;

  using pt_t = pcl::PointXYZ;
  using pc_t = pcl::PointCloud<pt_t>;
  using vec3_t = Eigen::Vector3f;

  using radians = mrs_lib::geometry::radians;

  class Bumper : public nodelet::Nodelet
  {
  public:
    /* onInit() method //{ */
    void onInit()
    {
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

      m_node_name = "Bumper";

      /* Load parameters from ROS //{*/
      mrs_lib::ParamLoader pl(nh, m_node_name);
      // LOAD STATIC PARAMETERS
      ROS_INFO("[Bumper]: Loading static parameters:");
      const auto uav_name = pl.loadParam2<std::string>("uav_name");
      pl.loadParam("update_rate", m_update_rate, 10.0);
      pl.loadParam("frame_id", m_frame_id);
      pl.loadParam("median_filter_size", m_median_filter_size, 1);

      pl.loadParam("depthmap/unknown_pixel_value", m_depthmap_unknown_pixel_value);
      m_depthmap_roi.x_offset = pl.loadParam2<int>("depthmap/roi/x_offset", 0);
      m_depthmap_roi.y_offset = pl.loadParam2<int>("depthmap/roi/y_offset", 0);
      m_depthmap_roi.width = pl.loadParam2<int>("depthmap/roi/width", 0);
      m_depthmap_roi.height = pl.loadParam2<int>("depthmap/roi/height", 0);
      pl.loadParam("depthmap/roi/centering", m_depthmap_roi_centering, false);
      pl.loadParam("depthmap/histogram_n_bins", m_depthmap_hist_n_bins, 1000);
      pl.loadParam("depthmap/histogram_quantile_area", m_depthmap_hist_quantile_area, 200);
      pl.loadParam("depthmap/max_depth", m_depthmap_max_depth);
      pl.loadParam("depthmap/camera_offset", m_depthmap_camera_offset);

      pl.loadParam("lidar3d/voxel_size", m_lidar3d_voxel_size);
      pl.loadParam("lidar3d/voxel_minpoints", m_lidar3d_voxel_minpoints);
      pl.loadParam("lidar3d/exclude_box/use", m_lidar3d_exclude_box_use);
      pl.loadMatrixStatic("lidar3d/exclude_box/offset", m_lidar3d_exclude_box_offset);
      pl.loadMatrixStatic("lidar3d/exclude_box/size", m_lidar3d_exclude_box_size);
      pl.loadParam("lidar3d/include_box/use", m_lidar3d_include_box_use);
      pl.loadMatrixStatic("lidar3d/include_box/offset", m_lidar3d_include_box_offset);
      pl.loadMatrixStatic("lidar3d/include_box/size", m_lidar3d_include_box_size);

      pl.loadParam("lidar2d/filter_size", m_lidar2d_filter_size);

      const auto fallback_timeout = pl.loadParam2<double>("fallback/timeout", 0.0);
      pl.loadParam("fallback/n_horizontal_sectors", m_fallback_n_horizontal_sectors, 0);
      pl.loadParam("fallback/vertical_fov", m_fallback_vertical_fov, 0.0);
      const auto path_to_mask = pl.loadParam2<std::string>("path_to_mask", std::string());

      // LOAD DYNAMIC PARAMETERS
      // CHECK LOADING STATUS
      if (!pl.loadedSuccessfully())
      {
        ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::requestShutdown();
      }

      m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);
      //}

      /* Create publishers and subscribers //{ */
      // Initialize subscribers
      mrs_lib::SubscribeHandlerOptions shopts;
      shopts.nh = nh;
      shopts.node_name = "Bumper";

      mrs_lib::construct_object(m_depthmap_sh, shopts, "depthmap_in");
      mrs_lib::construct_object(m_depth_cinfo_sh, shopts, "depth_cinfo_in");
      mrs_lib::construct_object(m_lidar3d_sh, shopts, "lidar3d_in");
      mrs_lib::construct_object(m_lidar2d_sh, shopts, "lidar2d_in");
      mrs_lib::construct_object(m_lidar1d_down_sh, shopts, "lidar1d_down_in");
      mrs_lib::construct_object(m_lidar1d_up_sh, shopts, "lidar1d_up_in");

      // Initialize publishers
      m_obstacles_pub = nh.advertise<ObstacleSectors>("obstacle_sectors", 1);
      m_processed_depthmap_pub = nh.advertise<sensor_msgs::Image>("processed_depthmap", 1);
      m_depthmap_hist_pub = nh.advertise<Histogram>("depthmap_histogram", 1);
      m_lidar3d_processed = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_processed", 1);

      m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
      //}

      /* Initialize other varibles //{ */
      if (path_to_mask.empty())
      {
        ROS_INFO("[%s]: Not using image mask", ros::this_node::getName().c_str());
      } else
      {
        m_depthmap_mask_im = cv::imread(path_to_mask, cv::IMREAD_GRAYSCALE);
        if (m_depthmap_mask_im.empty())
        {
          ROS_ERROR("[%s]: Error loading image mask from file '%s'! Ending node.", ros::this_node::getName().c_str(), path_to_mask.c_str());
          ros::requestShutdown();
        } else if (m_depthmap_mask_im.type() != CV_8UC1)
        {
          ROS_ERROR("[%s]: Loaded image mask has unexpected type: '%u' (expected %u)! Ending node.", ros::this_node::getName().c_str(), m_depthmap_mask_im.type(),
                    CV_8UC1);
          ros::requestShutdown();
        }
      }

      if (fallback_timeout != 0.0)
      {
        if (m_fallback_n_horizontal_sectors == 0 || m_fallback_vertical_fov == 0.0)
        {
          ROS_ERROR(
              "[%s]: Fallback timeout was specified, but fallback number of horizontal sectors (%d) or fallback vertical field of view (%.2f) is invalid (both "
              "have to be > 0). Ending node.",
              m_node_name.c_str(), m_fallback_n_horizontal_sectors, m_fallback_vertical_fov);
          ros::requestShutdown();
        }
      }
      m_fallback_timeout = ros::Duration(fallback_timeout);

      m_depthmap_roi_initialized = false;
      m_first_message_received = false;
      m_sectors_initialized = false;
      m_lidar2d_offset_initialized = false;

      m_tfm = mrs_lib::Transformer(m_node_name, uav_name);
      //}

      m_main_loop_timer = nh.createTimer(ros::Rate(m_update_rate), &Bumper::main_loop, this);

      std::cout << "----------------------------------------------------------" << std::endl;
    }
    //}

    /* main_loop() method //{ */
    void main_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      /* initialize some stuff, if possible //{ */
      
      /* Initialize number of horizontal sectors etc from camera info message //{ */
      if (!m_sectors_initialized && m_depth_cinfo_sh.hasMsg())
      {
        ROS_INFO("[Bumper]: Processing camera info message to initialize sectors");
      
        const auto cinfo = m_depth_cinfo_sh.getMsg();
        initialize_roi(cinfo->width, cinfo->height);
      
        const double w = m_depthmap_roi.width;
        const double h = m_depthmap_roi.height;
        const double fx = cinfo->K[0];
        const double fy = cinfo->K[4];
        const double horizontal_fov = std::atan2(w / 2.0, fx) * 2.0;
        const double vertical_fov = std::atan2(h / 2.0, fy) * 2.0;
        const int n_horizontal_sectors = std::ceil(2.0 * M_PI / horizontal_fov);
        initialize_sectors(n_horizontal_sectors, vertical_fov);
      
        ROS_INFO("[Bumper]: Depth camera horizontal FOV: %.1fdeg", horizontal_fov / M_PI * 180.0);
        ROS_INFO("[Bumper]: Depth camera vertical FOV: %.1fdeg", vertical_fov / M_PI * 180.0);
        ROS_INFO("[Bumper]: Number of horizontal sectors: %d", m_n_horizontal_sectors);
      }
      //}
      
      /* Initialize horizontal angle offset of 2D lidar from a new message //{ */
      if (!m_lidar2d_offset_initialized && m_lidar2d_sh.hasMsg())
      {
        ROS_INFO_THROTTLE(1.0, "[Bumper]: Initializing 2D lidar horizontal angle offset");
      
        const auto lidar2d_msg = m_lidar2d_sh.getMsg();
        initialize_lidar2d_offset(lidar2d_msg);
      
        if (m_lidar2d_offset_initialized)
          ROS_INFO("[Bumper]: 2D lidar horizontal angle offset: %.2f", m_lidar2d_offset);
        else
          ROS_WARN_THROTTLE(1.0, "[Bumper]: 2D lidar horizontal angle offset initialization failed, will retry.");
      }
      //}
      
      //}

      /* apply changes from dynamic reconfigure //{ */
      if (m_median_filter_size != m_drmgr_ptr->config.median_filter_size)
      {
        if (m_drmgr_ptr->config.median_filter_size > 0)
        {
          m_median_filter_size = m_drmgr_ptr->config.median_filter_size;
          update_filter_sizes();
        } else
        {
          ROS_ERROR("[Bumper]: Size of median filter cannot be <= 0: %d! Ignoring new value.", m_drmgr_ptr->config.median_filter_size);
        }
      }

      if (m_update_rate != m_drmgr_ptr->config.update_rate)
      {
        if (m_drmgr_ptr->config.update_rate > 0.0)
        {
          m_update_rate = m_drmgr_ptr->config.update_rate;
          m_main_loop_timer.setPeriod(ros::Duration(1.0 / m_update_rate));
        } else
        {
          ROS_ERROR("[Bumper]: Update rate cannot be <= 0: %lf! Ignoring new value.", m_drmgr_ptr->config.update_rate);
        }
      }
      //}

      if (m_sectors_initialized)
      {
        std::vector<std::tuple<int, std::vector<double>, ros::Time>> sensors_sectors;
        std::vector<std::string> sensors_topics;
        /* process any new messages from sensors //{ */

        // Check data from the depthmap image
        if (m_depthmap_sh.newMsg() && m_depth_cinfo_sh.hasMsg())
        {
          cv_bridge::CvImagePtr source_msg = cv_bridge::toCvCopy(m_depthmap_sh.getMsg(), std::string("16UC1"));
          if (!m_depthmap_roi_initialized)
            initialize_roi(source_msg->image.cols, source_msg->image.rows);
          const auto obstacle_sectors = find_obstacles_depthmap(source_msg);
          sensors_sectors.push_back({ObstacleSectors::SENSOR_DEPTH, obstacle_sectors, source_msg->header.stamp});
          sensors_topics.push_back(m_depthmap_sh.topicName());
        }

        // Check data from the 3D lidar
        if (m_lidar3d_sh.newMsg())
        {
          const auto cloud = m_lidar3d_sh.getMsg();
          std::vector<double> obstacle_sectors = find_obstacles_pointcloud(cloud);
          ros::Time msg_stamp;
          pcl_conversions::fromPCL(cloud->header.stamp, msg_stamp);
          sensors_sectors.push_back({ObstacleSectors::SENSOR_LIDAR3D, obstacle_sectors, msg_stamp});
          sensors_topics.push_back(m_lidar3d_sh.topicName());
        }

        // Check data from the horizontal 2D lidar
        if (m_lidar2d_offset_initialized && m_lidar2d_sh.newMsg())
        {
          const auto msg = m_lidar2d_sh.getMsg();
          std::vector<double> obstacle_sectors = find_obstacles_lidar2d(msg);
          sensors_sectors.push_back({ObstacleSectors::SENSOR_LIDAR2D, obstacle_sectors, msg->header.stamp});
          sensors_topics.push_back(m_lidar2d_sh.topicName());
        }

        // Check data from the down-facing lidar
        if (m_lidar1d_down_sh.newMsg())
        {
          const auto msg = m_lidar1d_down_sh.getMsg();
          std::vector<double> obstacle_sectors = find_obstacles_lidar1d(msg, m_bottom_sector_idx);
          sensors_sectors.push_back({ObstacleSectors::SENSOR_LIDAR1D, obstacle_sectors, msg->header.stamp});
          sensors_topics.push_back(m_lidar1d_down_sh.topicName());
        }

        // Check data from the up-facing lidar
        if (m_lidar1d_up_sh.newMsg())
        {
          const auto msg = m_lidar1d_up_sh.getMsg();
          std::vector<double> obstacle_sectors = find_obstacles_lidar1d(msg, m_top_sector_idx);
          sensors_sectors.push_back({ObstacleSectors::SENSOR_LIDAR1D, obstacle_sectors, msg->header.stamp});
          sensors_topics.push_back(m_lidar1d_up_sh.topicName());
        }

        //}

        std::vector<double> res_obstacles(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);
        std::vector<int8_t> res_sensors(m_n_total_sectors, ObstacleSectors::SENSOR_NONE);
        ros::Time res_stamp = ros::Time::now();
        /* put the resuls from different sensors together //{ */
        
        for (const auto& sensor_sectors : sensors_sectors)
        {
          const auto [sensor, sectors, stamp] = sensor_sectors;
          for (size_t sect_it = 0; sect_it < m_n_total_sectors; sect_it++)
          {
            const auto obstacle_dist = sectors.at(sect_it);
            // check if an obstacle was detected (*obstacle_sure*)
            const auto obstacle_unknown = obstacle_dist == ObstacleSectors::OBSTACLE_NO_DATA;
            auto& cur_value = res_obstacles.at(sect_it);
            auto  cur_unknown = cur_value == ObstacleSectors::OBSTACLE_NO_DATA;
            auto& cur_sensor = res_sensors.at(sect_it);
            // If the previous obstacle information in this sector is unknown or a closer
            // obstacle was detected by this sensor, update the information.
            if (!obstacle_unknown && (cur_value > obstacle_dist || cur_unknown))
            {
              cur_value = obstacle_dist;
              cur_sensor = sensor;
              if (res_stamp > stamp)
                res_stamp = stamp;
            }
          }
        }
        
        //}

        // filter the obstacles using a median filter
        res_obstacles = filter_sectors(res_obstacles);

        /* Prepare and publish the ObstacleSectors message to be published //{ */
        {
          ObstacleSectors obst_msg;
          obst_msg.header.frame_id = m_frame_id;
          obst_msg.header.stamp = res_stamp;
          obst_msg.n_horizontal_sectors = m_n_horizontal_sectors;
          obst_msg.sectors_vertical_fov = m_vertical_fov;
          obst_msg.sectors = res_obstacles;
          obst_msg.sector_sensors = res_sensors;
          m_obstacles_pub.publish(obst_msg);
        }
        //}

        /* print out some info to the console //{ */
        
        {
          std::vector<std::string> used_sensors;
          for (const auto& el : sensors_sectors)
          {
            const auto sensor = std::get<0>(el);
            switch (sensor)
            {
              case ObstacleSectors::SENSOR_NONE:
                used_sensors.push_back("\033[1;31minvalid \033[0m"); break;
              case ObstacleSectors::SENSOR_DEPTH:
                used_sensors.push_back("depthmap"); break;
              case ObstacleSectors::SENSOR_LIDAR1D:
                used_sensors.push_back("LiDAR 1D"); break;
              case ObstacleSectors::SENSOR_LIDAR2D:
                used_sensors.push_back("LiDAR 2D"); break;
              case ObstacleSectors::SENSOR_LIDAR3D:
                used_sensors.push_back("LiDAR 3D"); break;
            }
          }
          std::stringstream ss;
          for (size_t it = 0; it < used_sensors.size(); it++)
            ss << std::endl << "\t[" << used_sensors.at(it) << "] at topic \"" << sensors_topics.at(it) << "\"";
          ROS_INFO_STREAM_THROTTLE(2.0, "[Bumper]: Updating bumper using sensors:" << ss.str());
        }
        
        //}
      } else // if the number of sectors and vertical FOV were not initialized yet, check for fallback timeout
      {
        /* check for fallback timeout, apply if neccessary //{ */
        /* if we got a first sensor message, but still no cinfo, remember the stamp (for fallback timeout) //{ */

        if (!m_first_message_received && m_depthmap_sh.hasMsg())
        {
          m_first_message_stamp = m_depthmap_sh.getMsg()->header.stamp;
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar3d_sh.hasMsg())
        {
          pcl_conversions::fromPCL(m_lidar3d_sh.getMsg()->header.stamp, m_first_message_stamp);
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar2d_sh.hasMsg())
        {
          m_first_message_stamp = m_lidar2d_sh.getMsg()->header.stamp;
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar1d_down_sh.hasMsg())
        {
          m_first_message_stamp = m_lidar1d_down_sh.getMsg()->header.stamp;
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar1d_up_sh.hasMsg())
        {
          m_first_message_stamp = m_lidar1d_up_sh.getMsg()->header.stamp;
          m_first_message_received = true;
        }

        //}

        /* if the realsense timeout has run out, apply fallback values //{ */
        const ros::Duration cinfo_delay = ros::Time::now() - m_first_message_stamp;
        if (m_first_message_received && cinfo_delay >= m_fallback_timeout)
        {
          ROS_WARN("[%s]: No camera info message received after %.2f seconds, using fallback number of horizontal sectors: %d!", m_node_name.c_str(),
                   cinfo_delay.toSec(), m_fallback_n_horizontal_sectors);
          initialize_sectors(m_fallback_n_horizontal_sectors, m_fallback_vertical_fov);
          m_sectors_initialized = true;
        }
        //}
        //}
      }
    }
    //}

  private:
    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */
    double m_update_rate;
    std::string m_frame_id;
    int m_median_filter_size;

    int m_depthmap_unknown_pixel_value;
    sensor_msgs::RegionOfInterest m_depthmap_roi;
    bool m_depthmap_roi_centering;
    int m_depthmap_hist_n_bins;
    int m_depthmap_hist_quantile_area;
    double m_depthmap_max_depth;
    double m_depthmap_camera_offset;

    double m_lidar2d_filter_size;

    int m_lidar3d_voxel_minpoints;
    double m_lidar3d_voxel_size;
    bool m_lidar3d_exclude_box_use;
    Eigen::Vector3d m_lidar3d_exclude_box_offset;
    Eigen::Vector3d m_lidar3d_exclude_box_size;
    bool m_lidar3d_include_box_use;
    Eigen::Vector3d m_lidar3d_include_box_offset;
    Eigen::Vector3d m_lidar3d_include_box_size;

    ros::Duration m_fallback_timeout;
    int m_fallback_n_horizontal_sectors;
    double m_fallback_vertical_fov;
    //}

    /* ROS related variables (subscribers, timers etc.) //{ */
    std::unique_ptr<drmgr_t> m_drmgr_ptr;

    mrs_lib::SubscribeHandler<sensor_msgs::Image> m_depthmap_sh;
    mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> m_depth_cinfo_sh;
    mrs_lib::SubscribeHandler<pc_t> m_lidar3d_sh;
    mrs_lib::SubscribeHandler<sensor_msgs::LaserScan> m_lidar2d_sh;
    mrs_lib::SubscribeHandler<sensor_msgs::Range> m_lidar1d_down_sh;
    mrs_lib::SubscribeHandler<sensor_msgs::Range> m_lidar1d_up_sh;

    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

    ros::Publisher m_obstacles_pub;
    ros::Publisher m_processed_depthmap_pub;
    ros::Publisher m_depthmap_hist_pub;
    ros::Publisher m_lidar3d_processed;

    ros::Timer m_main_loop_timer;

    std::string m_node_name;
    //}

  private:
    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------

    /* Misc. member variables //{ */
    uint32_t m_n_horizontal_sectors;
    uint32_t m_bottom_sector_idx;
    uint32_t m_top_sector_idx;
    using angle_range_t = std::pair<double, double>;
    std::vector<angle_range_t> m_horizontal_sector_ranges;
    std::vector<boost::circular_buffer<double>> m_sector_filters;
    uint32_t m_n_total_sectors;
    double m_vertical_fov;
    float m_vertical_halffov_sin;

    cv::Mat m_depthmap_mask_im;
    bool m_depthmap_roi_initialized;
    ros::Time m_first_message_stamp;
    bool m_first_message_received;
    bool m_sectors_initialized;

    bool m_lidar2d_offset_initialized;
    double m_lidar2d_offset;
    //}

    mrs_lib::Transformer m_tfm;

  private:
    // --------------------------------------------------------------
    // |                 Obstacle detection methods                 |
    // --------------------------------------------------------------

    /* find_obstacles_depthmap() method //{ */
    std::vector<double> find_obstacles_depthmap(cv_bridge::CvImagePtr source_msg)
    {
      std::vector<double> ret(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);

      /* Apply ROI //{ */
      cv::Rect roi(m_depthmap_roi.x_offset, m_depthmap_roi.y_offset, m_depthmap_roi.width, m_depthmap_roi.height);
      source_msg->image = source_msg->image(roi);
      //}

      /* Prepare the image for obstacle detection //{ */
      // create the detection image
      cv::Mat detect_im = source_msg->image.clone();
      cv::Mat raw_im = source_msg->image;
      cv::Mat known_pixels;
      if (m_depthmap_unknown_pixel_value != std::numeric_limits<uint16_t>::max())
      {
        cv::compare(detect_im, m_depthmap_unknown_pixel_value, known_pixels, cv::CMP_NE);
      }

      // dilate and erode the image if requested
      {
        const int elem_a = m_drmgr_ptr->config.structuring_element_a;
        const int elem_b = m_drmgr_ptr->config.structuring_element_b;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(elem_a, elem_b), cv::Point(-1, -1));
        cv::dilate(detect_im, detect_im, element, cv::Point(-1, -1), m_drmgr_ptr->config.dilate_iterations);
        cv::erode(detect_im, detect_im, element, cv::Point(-1, -1), m_drmgr_ptr->config.erode_iterations);

        // erode without using zero (unknown) pixels
        if (m_drmgr_ptr->config.erode_ignore_empty_iterations > 0)
        {
          cv::Mat unknown_as_max = detect_im;
          if (m_depthmap_unknown_pixel_value != std::numeric_limits<uint16_t>::max())
          {
            unknown_as_max = cv::Mat(raw_im.size(), CV_16UC1, std::numeric_limits<uint16_t>::max());
            detect_im.copyTo(unknown_as_max, known_pixels);
          }
          cv::erode(unknown_as_max, detect_im, element, cv::Point(-1, -1), m_drmgr_ptr->config.erode_ignore_empty_iterations);
        }
      }
      //}

      // TODO: filter out ground?

      m_depthmap_hist_n_bins = m_drmgr_ptr->config.histogram_n_bins;
      m_depthmap_hist_quantile_area = m_drmgr_ptr->config.histogram_quantile_area;
      m_depthmap_max_depth = m_drmgr_ptr->config.max_depth;

      cv::Mat usable_pixels;
      if (m_depthmap_mask_im.empty())
        usable_pixels = known_pixels;
      else
        cv::bitwise_and(known_pixels, m_depthmap_mask_im, usable_pixels);

      const double depthmap_to_meters = 1.0 / 1000.0;
      const double bin_max = m_depthmap_max_depth / depthmap_to_meters;
      const double bin_size = bin_max / m_depthmap_hist_n_bins;
      const auto hist = calculate_histogram(detect_im, m_depthmap_hist_n_bins, bin_max, usable_pixels);
      const int quantile = find_histogram_quantile(hist, m_depthmap_hist_quantile_area);
      double obstacle_dist = quantile * bin_size * depthmap_to_meters;
      if (quantile == m_depthmap_hist_n_bins)
        obstacle_dist = ObstacleSectors::OBSTACLE_NOT_DETECTED;

      if (m_processed_depthmap_pub.getNumSubscribers() > 0)
      {
        /* Create and publish the debug image //{ */
        cv_bridge::CvImagePtr processed_depthmap_cvb = source_msg;
        processed_depthmap_cvb->image = detect_im;
        sensor_msgs::ImageConstPtr out_msg = processed_depthmap_cvb->toImageMsg();
        m_processed_depthmap_pub.publish(out_msg);
        //}
      }

      if (m_depthmap_hist_pub.getNumSubscribers() > 0)
      {
        /* Create and publish the debug image //{ */
        Histogram out_msg;
        out_msg.unit = "mm";
        out_msg.bin_size = bin_size;
        out_msg.bin_max = bin_max;
        out_msg.bin_min = 0;
        out_msg.bin_mark = quantile;
        out_msg.bins = hist;
        m_depthmap_hist_pub.publish(out_msg);
        //}
      }

      ret.at(0) = obstacle_dist;
      return ret;
    }
    //}

    /* find_obstacles_lidar1d() method //{ */
    std::vector<double> find_obstacles_lidar1d(const sensor_msgs::Range::ConstPtr& msg, const int sector)
    {
      std::vector<double> ret(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);

      double obstacle_dist = msg->range;
      if (obstacle_dist <= msg->min_range || obstacle_dist >= msg->max_range)
        obstacle_dist = ObstacleSectors::OBSTACLE_NOT_DETECTED;
      ret.at(sector) = obstacle_dist;
      
      return ret;
    }
    //}

    /* find_obstacles_lidar2d() method //{ */
    /* double buffer_max(const boost::circular_buffer<double>& buffer) */
    /* { */
    /*   double max = std::numeric_limits<double>::lowest(); */
    /*   for (const auto& val : buffer) */
    /*   { */
    /*     if (val > max) */
    /*       max = val; */
    /*   } */
    /*   return max; */
    /* } */

    std::vector<double> find_obstacles_lidar2d(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
      std::vector<double> ret(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);
      const auto buffer_length = m_lidar2d_filter_size;
      // check minimal obstacle distance for each horizontal sector
      for (size_t it = 0; it < m_n_horizontal_sectors; it++)
      {
        const auto& cur_angle_range = m_horizontal_sector_ranges.at(it);
        double min_range = std::numeric_limits<double>::max();
        // buffer of the last *buffer_length* measurements
        boost::circular_buffer<double> buffer(buffer_length);
        for (unsigned ray_it = 0; ray_it < scan_msg->ranges.size(); ray_it++)
        {
          const double ray_range = scan_msg->ranges.at(ray_it) + buffer_length / 2 * scan_msg->angle_increment;
          const double ray_angle = scan_msg->angle_min + ray_it * scan_msg->angle_increment + m_lidar2d_offset - buffer_length / 2 * scan_msg->angle_increment;
          // check if the ray is in the current horizontal sector
          if (angle_in_range(ray_angle, cur_angle_range))
          {
            buffer.push_back(ray_range);
            /* const double cur_max_range = buffer_max(buffer); */
            const double cur_max_range = *std::max_element(std::begin(buffer), std::end(buffer));
            // If the buffer has *buffer_length* measurements and maximal distance
            // in the buffer is lower than *min_range*, update *min_range*.
            // This should filter out solitary false detections of the laser rangefinder,
            // which would otherwise trigger the repulsion failsafe mechanism.
            if (buffer.full() && cur_max_range < min_range)
              min_range = cur_max_range;
          }
        }
        if (min_range < scan_msg->range_min || min_range > scan_msg->range_max)
          min_range = ObstacleSectors::OBSTACLE_NOT_DETECTED;
        ret.at(it) = min_range;
      }
      return ret;
    }
    //}

    /* find_obstacles_pointcloud() method //{ */
    std::vector<double> find_obstacles_pointcloud(const pc_t::ConstPtr& cloud_orig)
    {
      std::vector<double> ret(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);

      auto cloud = boost::make_shared<pc_t>();

      /* reduce the number of points using VoxelGrid (output to cloud) //{ */

      {
        pcl::VoxelGrid<pt_t> vg;
        vg.setInputCloud(cloud_orig);
        vg.setMinimumPointsNumberPerVoxel(m_lidar3d_voxel_minpoints);
        vg.setLeafSize(m_lidar3d_voxel_size, m_lidar3d_voxel_size, m_lidar3d_voxel_size);
        vg.filter(*cloud);
      }
      
      //}

      /* crop out points belonging to the UAV and too far points (include box and exclude box) //{ */
      
      {
        pcl::CropBox<pt_t> cb;

        if (m_lidar3d_exclude_box_use)
        {
          const Eigen::Vector3d box_point1 = m_lidar3d_exclude_box_offset + m_lidar3d_exclude_box_size/2.0;
          const Eigen::Vector3d box_point2 = m_lidar3d_exclude_box_offset - m_lidar3d_exclude_box_size/2.0;
          const Eigen::Vector4f bpt1(box_point1.x(), box_point1.y(), box_point1.z(), 1.0f);
          const Eigen::Vector4f bpt2(box_point2.x(), box_point2.y(), box_point2.z(), 1.0f);
          cb.setMax(bpt1);
          cb.setMin(bpt2);
          cb.setInputCloud(cloud);
          cb.setNegative(true);
          cb.filter(*cloud);
        }

        if (m_lidar3d_include_box_use)
        {
          const Eigen::Vector3d box_point1 = m_lidar3d_include_box_offset + m_lidar3d_include_box_size/2.0;
          const Eigen::Vector3d box_point2 = m_lidar3d_include_box_offset - m_lidar3d_include_box_size/2.0;
          const Eigen::Vector4f bpt1(box_point1.x(), box_point1.y(), box_point1.z(), 1.0f);
          const Eigen::Vector4f bpt2(box_point2.x(), box_point2.y(), box_point2.z(), 1.0f);
          cb.setMax(bpt1);
          cb.setMin(bpt2);
          cb.setInputCloud(cloud);
          cb.setNegative(false);
          cb.filter(*cloud);
        }
      }
      
      //}

      m_lidar3d_processed.publish(cloud);

      // transform the pointcloud to the untilted frame
      auto cloud_tfd_opt = m_tfm.transformSingle(m_frame_id, cloud);
      if (cloud_tfd_opt == std::nullopt)
        return ret;
      cloud = cloud_tfd_opt.value();

      for (const auto& el : *cloud)
      {
        const vec3_t ray(el.x, el.y, el.z);
        const auto [sector, dist] = sector_obstacle(ray);
        // if the obstacle isn't in any sector, skip the point
        if (sector < 0)
          continue;
        // otherwise, replace the minimum in the result list, if applicable
        auto& cur = ret.at(sector);
        if (cur == ObstacleSectors::OBSTACLE_NO_DATA || cur > dist)
          cur = dist;
      }
      
      return ret;
    }
    //}

  private:
    // --------------------------------------------------------------
    // |                       Helper methods                       |
    // --------------------------------------------------------------

    /* initialize_sectors() method //{ */
    // initializes some helper variables related to the sectors
    void initialize_sectors(int n_horizontal_sectors, double vfov)
    {
      m_n_horizontal_sectors = n_horizontal_sectors;
      m_bottom_sector_idx = n_horizontal_sectors;
      m_top_sector_idx = n_horizontal_sectors + 1;
      m_horizontal_sector_ranges = initialize_ranges(n_horizontal_sectors);
      m_n_total_sectors = n_horizontal_sectors + 2;
      m_vertical_fov = vfov;
      m_vertical_halffov_sin = std::sin(m_vertical_fov/2.0);
      update_filter_sizes();
      m_sectors_initialized = true;
    }
    //}

    /* initialize_ranges() method //{ */
    // initializes angle ranges of horizontal sectors
    std::vector<angle_range_t> initialize_ranges(uint32_t m_n_horizontal_sectors)
    {
      std::vector<angle_range_t> ret;
      ret.reserve(m_n_horizontal_sectors);
      for (unsigned sector_it = 0; sector_it < m_n_horizontal_sectors; sector_it++)
        ret.push_back(get_horizontal_sector_angle_interval(sector_it));
      return ret;
    }
    //}

    /* get_horizontal_sector_angle_interval() method //{ */
    // helper method for initialization of angle ranges of horizontal sectors
    angle_range_t get_horizontal_sector_angle_interval(unsigned sector_it)
    {
      assert(sector_it < m_n_horizontal_sectors);
      const double angle_step = 2.0 * M_PI / m_n_horizontal_sectors;
      const double angle_start = radians::wrap(sector_it * angle_step - angle_step / 2.0);
      const double angle_end = radians::wrap(angle_start + angle_step);
      return {angle_start, angle_end};
    }
    //}

    /* angle_in_range() method //{ */
    bool angle_in_range(double angle, const angle_range_t& angle_range)
    {
      angle = radians::wrap(angle);
      bool in_range = angle > angle_range.first && angle < angle_range.second;
      // corner-case for the first sector (which would have angle_range.first < 0.0, but it is normalized as angle_range.first + 2.0*M_PI)
      if (angle_range.first > angle_range.second)
        in_range = angle < angle_range.second || angle > angle_range.first;
      return in_range;
    }
    //}

    /* update_filter_sizes() method //{ */
    void update_filter_sizes()
    {
      const boost::circular_buffer<double> init_bfr(m_median_filter_size, ObstacleSectors::OBSTACLE_NO_DATA);
      m_sector_filters.resize(m_n_total_sectors, init_bfr);
      for (auto& fil : m_sector_filters)
        fil.rset_capacity(m_median_filter_size);
    }
    //}

    // | ------ helper methods for dephmap obstacle detection ----- |
    /* initialize_roi() method //{ */
    // initializes the image region of interest using parameters, loaded from ROS, and given dimensions of a received image 
    void initialize_roi(unsigned img_width, unsigned img_height)
    {
      if (m_depthmap_roi_centering)
      {
        if (m_depthmap_roi.height == 0)
          m_depthmap_roi.height = img_height;
        if (m_depthmap_roi.height > img_height)
        {
          ROS_ERROR("[%s]: Desired ROI height (%d) is larger than image height (%d) - clamping!", m_node_name.c_str(), m_depthmap_roi.height, img_height);
          m_depthmap_roi.height = img_height;
        }

        if (m_depthmap_roi.width == 0)
          m_depthmap_roi.width = img_width;
        if (m_depthmap_roi.width > img_width)
        {
          ROS_ERROR("[%s]: Desired ROI width (%d) is larger than image width (%d) - clamping!", m_node_name.c_str(), m_depthmap_roi.width, img_width);
          m_depthmap_roi.width = img_width;
        }

        m_depthmap_roi.y_offset = (img_height - m_depthmap_roi.height) / 2;
        m_depthmap_roi.x_offset = (img_width - m_depthmap_roi.width) / 2;
      }
      if (m_depthmap_roi.y_offset + m_depthmap_roi.height > unsigned(img_height) || m_depthmap_roi.height == 0)
        m_depthmap_roi.height = std::clamp(int(img_height - m_depthmap_roi.y_offset), 0, int(img_height));
      if (m_depthmap_roi.x_offset + m_depthmap_roi.width > unsigned(img_width) || m_depthmap_roi.width == 0)
        m_depthmap_roi.width = std::clamp(int(img_width), 0, int(img_width));
      m_depthmap_roi_initialized = true;
    }
    //}

    /* calculate_histogram() method //{ */
    using hist_t = std::vector<float>;
    hist_t calculate_histogram(const cv::Mat& img, const int n_bins, const double bin_max, const cv::Mat& mask)
    {
      assert(((img.type() & CV_MAT_DEPTH_MASK) == CV_16U) || ((img.type() & CV_MAT_DEPTH_MASK) == CV_16S));
      int histSize[] = {n_bins};
      float range[] = {0, (float)bin_max};
      const float* ranges[] = {range};
      int channels[] = {0};
      cv::MatND hist;
      calcHist(&img, 1, channels, mask, hist, 1, histSize, ranges);
      return hist;
    }
    //}

    /* find_histogram_quantile() method //{ */
    int find_histogram_quantile(const hist_t& hist, const double quantile_area)
    {
      double cur_area = 0;
      for (unsigned it = 0; it < hist.size(); it++)
      {
        cur_area += hist[it];
        if (cur_area > quantile_area)
          return it;
      }
      return hist.size();
    }
    //}

    // | ----- helper methods for 2D lidar obstacle detection ----- |
    /* initialize_lidar2d_offset() method //{ */
    // initializes an angle offset of a 2D LiDAR using transforms
    void initialize_lidar2d_offset(sensor_msgs::LaserScan::ConstPtr lidar2d_msg)
    {
      geometry_msgs::Vector3Stamped x_lidar;
      x_lidar.header = lidar2d_msg->header;
      x_lidar.vector.x = 1.0;
      x_lidar.vector.y = x_lidar.vector.z = 0.0;
      geometry_msgs::Vector3 x_fcu;
      const auto tfd_opt = m_tfm.transformSingle(m_frame_id, x_lidar);
      if (!tfd_opt.has_value())
        return;
      m_lidar2d_offset = std::atan2(tfd_opt->vector.y, tfd_opt->vector.x);
      m_lidar2d_offset_initialized = true;
    }
    //}

    // | ----- helper methods for 3D lidar obstacle detection ----- |
    /* sector_obstacle() method //{ */
    // rel_point: point, relative to the UAV (in the untilted coordinate frame)
    // returns: index of the sector in which the point lies and its distance
    std::pair<int, float> sector_obstacle(const vec3_t& rel_point)
    {
      const auto dist = rel_point.norm();
      const auto vert_sin = rel_point.z()/dist;
      if (vert_sin > m_vertical_halffov_sin)
      {
        return {m_top_sector_idx, dist};
      }
      else if (vert_sin < -m_vertical_halffov_sin)
      {
        return {m_bottom_sector_idx, dist};
      }
      else
      {
        const auto hori_angle = std::atan2(rel_point.y(), rel_point.x());

        for (size_t it = 0; it < m_n_horizontal_sectors; it++)
        {
          if (angle_in_range(hori_angle, m_horizontal_sector_ranges.at(it)))
            return {it, dist};
        }
      }
      return {ObstacleSectors::OBSTACLE_NO_DATA, ObstacleSectors::OBSTACLE_NO_DATA};
    }
    //}

    // | ----------- helper methods for median filtering ---------- |
    /* get_median() method //{ */
    template <typename T>
    static T get_median(const boost::circular_buffer<T>& buffer)
    {
      // copy the buffer to a helper vector
      std::vector<T> data;
      data.reserve(buffer.size());
      for (const auto& el : buffer)
        data.push_back(el);
      // sort the vector up to the nth element
      std::nth_element(std::begin(data), std::begin(data)+data.size()/2, std::end(data));
      // get the nth element (that's the median)
      const T median = *(std::begin(data)+data.size()/2);
      if (std::isinf(median))
        ROS_WARN("[Bumper]: median is inf...");
      return median;
    }
    //}

    /* print_median_buffer() method //{ */
    template <typename T>
    void print_median_buffer(boost::circular_buffer<T> fil, T median, unsigned n)
    {
      std::cout << "Buffer #" << n << ": " << median << std::endl;
      for (unsigned it = 0; it < fil.size(); it++)
        std::cout << it << "\t";
      std::cout << std::endl;
      for (const auto& v : fil)
        std::cout << std::fixed << std::setprecision(3) << v << "\t";
      std::cout << std::endl;
    }
    //}

    /* filter_sectors() method //{ */
    // applies one step of a median filter on each sector and returns the result
    template <typename T>
    std::vector<T> filter_sectors(const std::vector<T>& sectors)
    {
      assert(sectors.size() == m_sector_filters.size());
      std::vector<T> ret;
      ret.reserve(sectors.size());
      for (unsigned it = 0; it < sectors.size(); it++)
      {
        const T sec = sectors.at(it);
        auto& fil = m_sector_filters.at(it);
        fil.push_back(sec);
        const auto median = get_median(fil);
        ret.push_back(median);
#ifdef DEBUG_MEDIAN_FILTER
        print_median_buffer(fil, median, it);
#endif
      }
      return ret;
    }
    //}

  };  // class Bumper
};    // namespace mrs_bumper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_bumper::Bumper, nodelet::Nodelet)

