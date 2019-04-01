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

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/DynamicReconfigureMgr.h>
#include <mrs_lib/SubscribeHandler.h>

#include <mrs_bumper/BumperConfig.h>
#include <mrs_msgs/ObstacleSectors.h>
#include <mrs_msgs/Histogram.h>

// shortcut type to the dynamic reconfigure manager template instance
typedef mrs_lib::DynamicReconfigureMgr<mrs_bumper::BumperConfig> drmgr_t;

namespace mrs_bumper
{

  using ObstacleSectors = mrs_msgs::ObstacleSectors;
  using Histogram = mrs_msgs::Histogram;

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
      pl.load_param("update_rate", m_update_rate, 10.0);
      pl.load_param("unknown_pixel_value", m_unknown_pixel_value);
      pl.load_param("frame_id", m_frame_id);
      pl.load_param("median_filter_size", m_median_filter_size, 1);
      m_roi.x_offset = pl.load_param2<int>("roi/x_offset", 0);
      m_roi.y_offset = pl.load_param2<int>("roi/y_offset", 0);
      m_roi.width = pl.load_param2<int>("roi/width", 0);
      m_roi.height = pl.load_param2<int>("roi/height", 0);
      pl.load_param("roi/centering", m_roi_centering, false);
      pl.load_param("histogram_n_bins", m_hist_n_bins, 1000);
      pl.load_param("histogram_quantile_area", m_hist_quantile_area, 200);
      pl.load_param("max_depth", m_max_depth);
      pl.load_param("depth_camera_offset", m_depth_camera_offset);
      const double fallback_timeout = pl.load_param2<double>("fallback_timeout", 0.0);
      pl.load_param("fallback_n_horizontal_sectors", m_fallback_n_horizontal_sectors, 0);
      pl.load_param("fallback_vertical_fov", m_fallback_vertical_fov, 0.0);
      const std::string path_to_mask = pl.load_param2<std::string>("path_to_mask", std::string());

      // LOAD DYNAMIC PARAMETERS
      // CHECK LOADING STATUS
      if (!pl.loaded_successfully())
      {
        ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
        ros::shutdown();
      }

      m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);
      //}

      /* Create publishers and subscribers //{ */
      // Initialize subscribers
      mrs_lib::SubscribeMgr smgr(nh, m_node_name);
      const bool subs_time_consistent = false;
      m_depthmap_sh = smgr.create_handler_threadsafe<sensor_msgs::ImageConstPtr, subs_time_consistent>("depthmap", 1, ros::TransportHints().tcpNoDelay(),
                                                                                                       ros::Duration(5.0));
      m_depth_cinfo_sh = smgr.create_handler_threadsafe<sensor_msgs::CameraInfoConstPtr, subs_time_consistent>(
          "depth_camera_info", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      m_lidar_2d_sh = smgr.create_handler_threadsafe<sensor_msgs::LaserScanConstPtr, subs_time_consistent>("lidar_2d", 1, ros::TransportHints().tcpNoDelay(),
                                                                                                           ros::Duration(5.0));
      m_lidar_1d_down_sh = smgr.create_handler_threadsafe<sensor_msgs::RangeConstPtr, subs_time_consistent>(
          "lidar_1d_down", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      m_lidar_1d_up_sh = smgr.create_handler_threadsafe<sensor_msgs::RangeConstPtr, subs_time_consistent>("lidar_1d_up", 1, ros::TransportHints().tcpNoDelay(),
                                                                                                          ros::Duration(5.0));
      // Initialize publishers
      m_obstacles_pub = nh.advertise<ObstacleSectors>("obstacle_sectors", 1);
      m_processed_depthmap_pub = nh.advertise<sensor_msgs::Image>("processed_depthmap", 1);
      m_depthmap_hist_pub = nh.advertise<Histogram>("depthmap_histogram", 1);

      m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
      //}

      /* Initialize other varibles //{ */
      if (path_to_mask.empty())
      {
        ROS_INFO("[%s]: Not using image mask", ros::this_node::getName().c_str());
      } else
      {
        m_mask_im = cv::imread(path_to_mask, cv::IMREAD_GRAYSCALE);
        if (m_mask_im.empty())
        {
          ROS_ERROR("[%s]: Error loading image mask from file '%s'! Ending node.", ros::this_node::getName().c_str(), path_to_mask.c_str());
          ros::shutdown();
        } else if (m_mask_im.type() != CV_8UC1)
        {
          ROS_ERROR("[%s]: Loaded image mask has unexpected type: '%u' (expected %u)! Ending node.", ros::this_node::getName().c_str(), m_mask_im.type(),
                    CV_8UC1);
          ros::shutdown();
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
        }
      }
      m_fallback_timeout = ros::Duration(fallback_timeout);

      m_roi_initialized = false;
      m_first_message_received = false;
      m_sectors_initialized = false;
      m_lidar_2d_offset_initialized = false;
      //}

      m_main_loop_timer = nh.createTimer(ros::Rate(m_update_rate), &Bumper::main_loop, this);

      std::cout << "----------------------------------------------------------" << std::endl;
    }
    //}

    /* main_loop() method //{ */
    void main_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      /* Initialize number of horizontal sectors etc from camera info message //{ */
      if (!m_sectors_initialized && m_depth_cinfo_sh->has_data())
      {
        ROS_INFO("[Bumper]: Processing camera info message to initialize sectors");

        const auto cinfo = m_depth_cinfo_sh->get_data();
        initialize_roi(cinfo->width, cinfo->height);

        const double w = m_roi.width;
        const double h = m_roi.height;
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
      if (!m_lidar_2d_offset_initialized && m_lidar_2d_sh->has_data())
      {
        ROS_INFO("[Bumper]: Initializing 2D lidar horizontal angle offset");

        const auto lidar_2d_msg = m_lidar_2d_sh->get_data();
        initialize_lidar_2d_offset(lidar_2d_msg);

        if (m_lidar_2d_offset_initialized)
          ROS_INFO("[Bumper]: 2D lidar horizontal angle offset: %.2f", m_lidar_2d_offset);
        else
          ROS_INFO("[Bumper]: 2D lidar horizontal angle offset initialization failed, will retry.");
      }
      //}

      /* Apply possible changes from dynamic reconfigure //{ */
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
        /* process any new messages, publish new obstacles message //{ */

        /* Prepare the ObstacleSectors message to be published //{ */
        ObstacleSectors obst_msg;
        obst_msg.header.frame_id = m_frame_id;
        obst_msg.header.stamp = ros::Time::now();
        obst_msg.n_horizontal_sectors = m_n_horizontal_sectors;
        obst_msg.sectors_vertical_fov = m_vertical_fov;
        obst_msg.sectors.resize(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);
        obst_msg.sector_sensors.resize(m_n_total_sectors, ObstacleSectors::SENSOR_NONE);
        //}

        /* Check data from the front-facing realsense //{ */
        if (m_depthmap_sh->new_data() && m_depth_cinfo_sh->has_data())
        {
          cv_bridge::CvImagePtr source_msg = cv_bridge::toCvCopy(m_depthmap_sh->get_data(), std::string("16UC1"));
          if (!m_roi_initialized)
            initialize_roi(source_msg->image.cols, source_msg->image.rows);
          double obstacle_dist = find_obstacles_in_depthmap(source_msg);

          // check if an obstacle was detected (*obstacle_sure*)
          bool obstacle_sure = !value_is_unknown(obstacle_dist);
          if (obstacle_sure)
            obstacle_dist += m_depth_camera_offset;
          auto& cur_value = obst_msg.sectors.at(0);
          auto& cur_sensor = obst_msg.sector_sensors.at(0);
          // If the previous obstacle information in this sector is unknown or a closer
          // obstacle was detected by this sensor, update the information.
          // TODO: fix the logic here
          if (value_is_unknown(cur_value) || (obstacle_sure && obstacle_dist < cur_value))
          {
            cur_value = obstacle_dist;
            cur_sensor = ObstacleSectors::SENSOR_DEPTH;
          }
          if (obst_msg.header.stamp > source_msg->header.stamp)
            obst_msg.header.stamp = source_msg->header.stamp;
        }
        //}

        /* Check data from the horizontal 2D lidar //{ */
        if (m_lidar_2d_offset_initialized && m_lidar_2d_sh->new_data())
        {
          sensor_msgs::LaserScan source_msg = *m_lidar_2d_sh->get_data();

          std::vector<double> obstacle_distances = find_obstacles_in_horizontal_sectors(source_msg);
          for (unsigned sector_it = 0; sector_it < m_n_horizontal_sectors; sector_it++)
          {
            // get the current obstacle distance
            const double obstacle_dist = obstacle_distances.at(sector_it);

            // check if an obstacle was detected (*obstacle_sure*)
            bool obstacle_sure = !value_is_unknown(obstacle_dist);
            auto& cur_value = obst_msg.sectors.at(sector_it);
            auto& cur_sensor = obst_msg.sector_sensors.at(sector_it);
            // If the previous obstacle information in this sector is unknown or a closer
            // obstacle was detected by this sensor, update the information.
            if (value_is_unknown(cur_value) || (obstacle_sure && obstacle_dist < cur_value))
            {
              cur_value = obstacle_dist;
              cur_sensor = ObstacleSectors::SENSOR_LIDAR_2D;
            }
            if (obst_msg.header.stamp > source_msg.header.stamp)
              obst_msg.header.stamp = source_msg.header.stamp;
          }
        }
        //}

        /* Check data from the down-facing lidar //{ */
        if (m_lidar_1d_down_sh->new_data())
        {
          // get the current obstacle distance
          sensor_msgs::Range source_msg = *m_lidar_1d_down_sh->get_data();
          double obstacle_dist = source_msg.range;
          if (obstacle_dist <= source_msg.min_range || obstacle_dist >= source_msg.max_range)
            obstacle_dist = ObstacleSectors::OBSTACLE_NOT_DETECTED;

          // check if an obstacle was detected (*obstacle_sure*)
          bool obstacle_sure = !value_is_unknown(obstacle_dist);
          auto& cur_value = obst_msg.sectors.at(m_bottom_sector_idx);
          auto& cur_sensor = obst_msg.sector_sensors.at(m_bottom_sector_idx);
          // If the previous obstacle information in this sector is unknown or a closer
          // obstacle was detected by this sensor, update the information.
          if (value_is_unknown(cur_value) || (obstacle_sure && obstacle_dist < cur_value))
          {
            cur_value = obstacle_dist;
            cur_sensor = ObstacleSectors::SENSOR_LIDAR_1D;
          }
          if (obst_msg.header.stamp > source_msg.header.stamp)
            obst_msg.header.stamp = source_msg.header.stamp;
        }
        //}

        /* Check data from the up-facing lidar //{ */
        if (m_lidar_1d_up_sh->new_data())
        {
          // get the current obstacle distance
          sensor_msgs::Range source_msg = *m_lidar_1d_up_sh->get_data();
          double obstacle_dist = source_msg.range;
          if (obstacle_dist <= source_msg.min_range || obstacle_dist >= source_msg.max_range)
            obstacle_dist = ObstacleSectors::OBSTACLE_NOT_DETECTED;

          // check if an obstacle was detected (*obstacle_sure*)
          bool obstacle_sure = !value_is_unknown(obstacle_dist);
          auto& cur_value = obst_msg.sectors.at(m_top_sector_idx);
          auto& cur_sensor = obst_msg.sector_sensors.at(m_top_sector_idx);
          // If the previous obstacle information in this sector is unknown or a closer
          // obstacle was detected by this sensor, update the information.
          if (value_is_unknown(cur_value) || (obstacle_sure && obstacle_dist < cur_value))
          {
            cur_value = obstacle_dist;
            cur_sensor = ObstacleSectors::SENSOR_LIDAR_1D;
          }
          if (obst_msg.header.stamp > source_msg.header.stamp)
            obst_msg.header.stamp = source_msg.header.stamp;
        }
        //}

        /* filter the obstacle distances //{ */

        obst_msg.sectors = filter_sectors(obst_msg.sectors);

        //}

        /* Publish the ObstacleSectors message //{ */
        m_obstacles_pub.publish(obst_msg);
        //}

        //}
      } else
      {
        /* check for fallback timeout, apply if neccessary //{ */
        /* if we got a first sensor message, but still no cinfo, remember the stamp (for fallback timeout) //{ */
        
        if (!m_first_message_received && m_depthmap_sh->has_data())
        {
          m_first_message_stamp = m_depthmap_sh->get_data()->header.stamp;
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar_2d_sh->has_data())
        {
          m_first_message_stamp = m_lidar_2d_sh->get_data()->header.stamp;
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar_1d_down_sh->has_data())
        {
          m_first_message_stamp = m_lidar_1d_down_sh->get_data()->header.stamp;
          m_first_message_received = true;
        }
        if (!m_first_message_received && m_lidar_1d_up_sh->has_data())
        {
          m_first_message_stamp = m_lidar_1d_up_sh->get_data()->header.stamp;
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
    int m_unknown_pixel_value;
    std::string m_frame_id;
    int m_median_filter_size;
    sensor_msgs::RegionOfInterest m_roi;
    bool m_roi_centering;
    int m_hist_n_bins;
    int m_hist_quantile_area;
    double m_max_depth;
    double m_depth_camera_offset;

    ros::Duration m_fallback_timeout;
    int m_fallback_n_horizontal_sectors;
    double m_fallback_vertical_fov;
    //}

    /* ROS related variables (subscribers, timers etc.) //{ */
    std::unique_ptr<drmgr_t> m_drmgr_ptr;

    mrs_lib::SubscribeHandlerPtr<sensor_msgs::ImageConstPtr> m_depthmap_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::CameraInfoConstPtr> m_depth_cinfo_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::LaserScanConstPtr> m_lidar_2d_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::RangeConstPtr> m_lidar_1d_down_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::RangeConstPtr> m_lidar_1d_up_sh;

    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

    ros::Publisher m_obstacles_pub;
    ros::Publisher m_processed_depthmap_pub;
    ros::Publisher m_depthmap_hist_pub;

    ros::Timer m_main_loop_timer;

    std::string m_node_name;
    //}

  private:
    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------

    /* Misc. member variables //{ */
    cv::Mat m_mask_im;
    uint32_t m_n_horizontal_sectors;
    uint32_t m_bottom_sector_idx;
    uint32_t m_top_sector_idx;
    using angle_range_t = std::pair<double, double>;
    std::vector<angle_range_t> m_horizontal_sector_ranges;
    std::vector<boost::circular_buffer<double>> m_sector_filters;
    uint32_t m_n_total_sectors;
    double m_vertical_fov;
    bool m_roi_initialized;
    ros::Time m_first_message_stamp;
    bool m_first_message_received;
    bool m_sectors_initialized;
    bool m_lidar_2d_offset_initialized;
    double m_lidar_2d_offset;
    //}

  private:
    // --------------------------------------------------------------
    // |                       Helper methods                       |
    // --------------------------------------------------------------

    /* value_is_unknown() method //{ */
    bool value_is_unknown(ObstacleSectors::_sectors_type::value_type value)
    {
      return value == ObstacleSectors::OBSTACLE_NO_DATA || value == ObstacleSectors::OBSTACLE_NOT_DETECTED;
    }
    //}

    /* initialize_roi() method //{ */
    void initialize_roi(unsigned img_width, unsigned img_height)
    {
      if (m_roi_centering)
      {
        if (m_roi.height == 0)
          m_roi.height = img_height;
        if (m_roi.height > img_height)
        {
          ROS_ERROR("[%s]: Desired ROI height (%d) is larger than image height (%d) - clamping!", m_node_name.c_str(), m_roi.height, img_height);
          m_roi.height = img_height;
        }

        if (m_roi.width == 0)
          m_roi.width = img_width;
        if (m_roi.width > img_width)
        {
          ROS_ERROR("[%s]: Desired ROI width (%d) is larger than image width (%d) - clamping!", m_node_name.c_str(), m_roi.width, img_width);
          m_roi.width = img_width;
        }

        m_roi.y_offset = (img_height - m_roi.height) / 2;
        m_roi.x_offset = (img_width - m_roi.width) / 2;
      }
      if (m_roi.y_offset + m_roi.height > unsigned(img_height) || m_roi.height == 0)
        m_roi.height = std::clamp(int(img_height - m_roi.y_offset), 0, int(img_height));
      if (m_roi.x_offset + m_roi.width > unsigned(img_width) || m_roi.width == 0)
        m_roi.width = std::clamp(int(img_width), 0, int(img_width));
      m_roi_initialized = true;
    }
    //}

    /* initialize_lidar_2d_offset() method //{ */
    void initialize_lidar_2d_offset(sensor_msgs::LaserScan::ConstPtr lidar_2d_msg)
    {
      tf2::Vector3 x_lidar(1.0, 0.0, 0.0);
      tf2::Vector3 x_fcu;
      if (!transform(x_lidar, lidar_2d_msg->header.frame_id, x_fcu, m_frame_id, lidar_2d_msg->header.stamp))
        return;
      m_lidar_2d_offset = atan2(x_fcu.getY(), x_fcu.getX());
      m_lidar_2d_offset_initialized = true;
    }
    //}

    /* initialize_sectors() method //{ */
    void initialize_sectors(int n_horizontal_sectors, double vfov)
    {
      m_n_horizontal_sectors = n_horizontal_sectors;
      m_bottom_sector_idx = n_horizontal_sectors;
      m_top_sector_idx = n_horizontal_sectors + 1;
      m_horizontal_sector_ranges = initialize_ranges(n_horizontal_sectors);
      m_n_total_sectors = n_horizontal_sectors + 2;
      m_vertical_fov = vfov;
      update_filter_sizes();
      m_sectors_initialized = true;
    }
    //}

    /* transform() method //{ */
    template <typename T>
    bool transform(const T& orig, const std::string& from_frame_id, T& transformed, const std::string& to_frame_id, const ros::Time& stamp)
    {
      try
      {
        const ros::Duration timeout(1.0 / 100.0);
        // Obtain transform between the frames
        const geometry_msgs::TransformStamped tf = m_tf_buffer.lookupTransform(to_frame_id, from_frame_id, stamp, timeout);
        tf2::doTransform(orig, transformed, tf);
      }
      catch (tf2::TransformException& ex)
      {
        NODELET_WARN_THROTTLE(1.0, "[%s]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", m_node_name.c_str(), from_frame_id.c_str(),
                              to_frame_id.c_str(), ex.what());
        return false;
      }
      return true;
    }
    //}

    using hist_t = std::vector<float>;
    /* calculate_histogram() method //{ */
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

    /* find_obstacles_in_depthmap() method //{ */
    double find_obstacles_in_depthmap(cv_bridge::CvImagePtr source_msg)
    {
      /* Apply ROI //{ */
      cv::Rect roi(m_roi.x_offset, m_roi.y_offset, m_roi.width, m_roi.height);
      source_msg->image = source_msg->image(roi);
      //}

      /* Prepare the image for obstacle detection //{ */
      // create the detection image
      cv::Mat detect_im = source_msg->image.clone();
      cv::Mat raw_im = source_msg->image;
      cv::Mat known_pixels;
      if (m_unknown_pixel_value != std::numeric_limits<uint16_t>::max())
      {
        cv::compare(detect_im, m_unknown_pixel_value, known_pixels, cv::CMP_NE);
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
          if (m_unknown_pixel_value != std::numeric_limits<uint16_t>::max())
          {
            unknown_as_max = cv::Mat(raw_im.size(), CV_16UC1, std::numeric_limits<uint16_t>::max());
            detect_im.copyTo(unknown_as_max, known_pixels);
          }
          cv::erode(unknown_as_max, detect_im, element, cv::Point(-1, -1), m_drmgr_ptr->config.erode_ignore_empty_iterations);
        }
      }
      //}

      // TODO: filter out ground?

      m_hist_n_bins = m_drmgr_ptr->config.histogram_n_bins;
      m_hist_quantile_area = m_drmgr_ptr->config.histogram_quantile_area;
      m_max_depth = m_drmgr_ptr->config.max_depth;

      cv::Mat usable_pixels;
      if (m_mask_im.empty())
        usable_pixels = known_pixels;
      else
        cv::bitwise_and(known_pixels, m_mask_im, usable_pixels);

      const double depthmap_to_meters = 1.0 / 1000.0;
      const double bin_max = m_max_depth / depthmap_to_meters;
      const double bin_size = bin_max / m_hist_n_bins;
      const auto hist = calculate_histogram(detect_im, m_hist_n_bins, bin_max, usable_pixels);
#ifdef DEBUG
      show_hist(hist);
#endif
      const int quantile = find_histogram_quantile(hist, m_hist_quantile_area);
      double obstacle_dist = quantile * bin_size * depthmap_to_meters;
      if (quantile == m_hist_n_bins)
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

      return obstacle_dist;
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

    /* normalize_angle() method //{ */
    double normalize_angle(double angle)
    {
      if (angle < 0.0)
        angle += 2.0 * M_PI;
      else if (angle >= 2.0 * M_PI)
        angle -= 2.0 * M_PI;
      return angle;
    }
    //}

    /* get_horizontal_sector_angle_interval() method //{ */
    angle_range_t get_horizontal_sector_angle_interval(unsigned sector_it)
    {
      assert(sector_it > 0 && sector_it < m_n_horizontal_sectors);
      const double angle_step = 2.0 * M_PI / m_n_horizontal_sectors;
      const double angle_start = normalize_angle(sector_it * angle_step - angle_step / 2.0);
      const double angle_end = normalize_angle(angle_start + angle_step);
      return {angle_start, angle_end};
    }
    //}

    /* initialize_ranges() method //{ */
    std::vector<angle_range_t> initialize_ranges(uint32_t m_n_horizontal_sectors)
    {
      std::vector<angle_range_t> ret;
      ret.reserve(m_n_horizontal_sectors);
      for (unsigned sector_it = 0; sector_it < m_n_horizontal_sectors; sector_it++)
        ret.push_back(get_horizontal_sector_angle_interval(sector_it));
      return ret;
    }
    //}

    /* angle_in_range() method //{ */
    bool angle_in_range(double angle, const angle_range_t& angle_range)
    {
      angle = normalize_angle(angle);
      bool in_range = angle > angle_range.first && angle < angle_range.second;
      // corner-case for the first sector (which would have angle_range.first < 0.0, but it is normalized as angle_range.first + 2.0*M_PI)
      if (angle_range.first > angle_range.second)
        in_range = angle < angle_range.second || angle > angle_range.first;
      return in_range;
    }
    //}

    /* find_obstacles_in_horizontal_sectors() method //{ */
    using scan_cit_t = sensor_msgs::LaserScan::_ranges_type::const_iterator;
    std::vector<double> find_obstacles_in_horizontal_sectors(const sensor_msgs::LaserScan& scan_msg)
    {
      std::vector<double> ret;
      ret.reserve(m_n_horizontal_sectors);
      for (const auto& cur_angle_range : m_horizontal_sector_ranges)
      {
        double min_range = std::numeric_limits<double>::max();
        for (unsigned ray_it = 0; ray_it < scan_msg.ranges.size(); ray_it++)
        {
          const double ray_range = scan_msg.ranges.at(ray_it);
          const double ray_angle = scan_msg.angle_min + ray_it * scan_msg.angle_increment + m_lidar_2d_offset;
          if (ray_range < min_range && angle_in_range(ray_angle, cur_angle_range))
            min_range = ray_range;
        }
        if (min_range < scan_msg.range_min || min_range > scan_msg.range_max)
          min_range = ObstacleSectors::OBSTACLE_NOT_DETECTED;
        ret.push_back(min_range);
      }
      return ret;
    }
    //}

    /* find_min_larger() method //{ */
    // finds the smallest unused value in *buf*, which is greater of equal to *ge_to*
    // updating the *used* variable to indicate which element was used
    template <typename T>
    static T find_unused_min_ge(const boost::circular_buffer<T>& buf, const T ge_to, std::vector<int>& used)
    {
      T ret = std::numeric_limits<T>::max();
      int used_it = -1;
      for (unsigned it = 0; it < buf.size(); it++)
      {
        int& el_used = used.at(it);
        if (el_used)
          continue;
        const T& val = buf.at(it);
        if (val < ret && val >= ge_to)
        {
          ret = val;
          used_it = it;
        }
      }
      if (used_it >= 0)
        used.at(used_it) = 1;
      return ret;
    }
    //}

    /* get_median() method //{ */
    template <typename T>
    static T get_median(const boost::circular_buffer<T>& filter)
    {
      const unsigned len = filter.size();
      /* const bool even_len = filter.size() % 2 == 0; */
      T prev_min = -std::numeric_limits<T>::infinity();
      std::vector<int> used(len, 0);
      const unsigned it_max = std::max(len / 2u, 1u);
      for (unsigned it = 0; it < it_max; it++)
        prev_min = find_unused_min_ge(filter, prev_min, used);
      T median = prev_min;
      /* if (even_len) */
      /* { */
      /*   T median2 = find_unused_min_ge(filter, prev_min, used); */
      /*   median = (median + median2) / T(2); */
      /* } */
      if (std::isinf(median))
        ROS_WARN("[Bumper]: median is inf...");
      return median;
    }
    //}

    /* filter_sectors() method //{ */
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

