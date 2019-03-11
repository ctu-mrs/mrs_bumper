#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/DynamicReconfigureMgr.h>
#include <mrs_lib/SubscribeHandler.h>

#include <mrs_bumper/BumperConfig.h>
#include <mrs_bumper/ObstacleSectors.h>

// shortcut type to the dynamic reconfigure manager template instance
typedef mrs_lib::DynamicReconfigureMgr<mrs_bumper::BumperConfig> drmgr_t;

namespace mrs_bumper
{

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
      ROS_INFO("Loading static parameters:");
      double update_rate = pl.load_param2<double>("update_rate", 10.0);
      pl.load_param("unknown_pixel_value", m_unknown_pixel_value);
      pl.load_param("frame_id", m_frame_id);
      m_roi.x_offset = pl.load_param2<int>("roi/x_offset", 0);
      m_roi.y_offset = pl.load_param2<int>("roi/y_offset", 0);
      m_roi.width = pl.load_param2<int>("roi/width", 0);
      m_roi.height = pl.load_param2<int>("roi/height", 0);
      pl.load_param<bool>("roi/centering", m_roi_centering, false);
      std::string path_to_mask = pl.load_param2<std::string>("path_to_mask", std::string());

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
      m_depthmap_sh = smgr.create_handler_threadsafe<sensor_msgs::ImageConstPtr, subs_time_consistent>("depthmap", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      m_depth_cinfo_sh = smgr.create_handler_threadsafe<sensor_msgs::CameraInfoConstPtr, subs_time_consistent>("depth_camera_info", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      m_lidar_2d_sh = smgr.create_handler_threadsafe<sensor_msgs::LaserScanConstPtr, subs_time_consistent>("lidar_2d", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      m_lidar_1d_down_sh = smgr.create_handler_threadsafe<sensor_msgs::RangeConstPtr, subs_time_consistent>("lidar_1d_down", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      m_lidar_1d_up_sh = smgr.create_handler_threadsafe<sensor_msgs::RangeConstPtr, subs_time_consistent>("lidar_1d_up", 1, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));
      // Initialize publishers
      m_obstacles_pub = nh.advertise<mrs_bumper::ObstacleSectors>("obstacle_sectors", 1);
      m_processed_depthmap_pub = nh.advertise<sensor_msgs::Image>("processed_depthmap", 1);
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
          ROS_ERROR("[%s]: Loaded image mask has unexpected type: '%u' (expected %u)! Ending node.", ros::this_node::getName().c_str(), m_mask_im.type(), CV_8UC1);
          ros::shutdown();
        }
      }

      m_sectors_initialized = false;
      //}

      m_main_loop_timer = nh.createTimer(ros::Rate(update_rate), &Bumper::main_loop, this);

      std::cout << "----------------------------------------------------------" << std::endl;

    }
    //}

    /* main_loop() method //{ */
    void main_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      /* Initialize number of horizontal sectors etc from camera info message //{ */
      if (!m_sectors_initialized && m_depth_cinfo_sh->new_data())
      {
        const auto cinfo = m_depth_cinfo_sh->get_data();
        if (m_roi_centering)
        {
          if (m_roi.height == 0)
            m_roi.height = cinfo->height;
          if (m_roi.width == 0)
            m_roi.width = cinfo->width;

          m_roi.y_offset = (cinfo->height-m_roi.height)/2;
          m_roi.x_offset = (cinfo->width-m_roi.width)/2;
        }


        if (m_roi.y_offset + m_roi.height > unsigned(cinfo->height) || m_roi.height == 0)
          m_roi.height = std::clamp(int(cinfo->height - m_roi.y_offset), 0, int(cinfo->height));
        if (m_roi.x_offset + m_roi.width > unsigned(cinfo->width) || m_roi.width == 0)
          m_roi.width = std::clamp(int(cinfo->width - m_roi.x_offset), 0, int(cinfo->width));
        
        const double w = m_roi.width;
        const double h = m_roi.height;
        const double fx = cinfo->K[0];
        const double fy = cinfo->K[4];
        const double horizontal_fov = std::atan2(w/2.0, fx)*2.0;
        const double vectical_fov = std::atan2(h/2.0, fy)*2.0;
        m_n_horizontal_sectors = std::ceil(2.0*M_PI/horizontal_fov);
        m_bottom_sector_idx = m_n_horizontal_sectors;
        m_top_sector_idx = m_n_horizontal_sectors+1;
        m_horizontal_sector_ranges = initialize_ranges(m_n_horizontal_sectors);
        m_n_total_sectors = m_n_horizontal_sectors + 2;
        m_vertical_fov = vectical_fov;
        ROS_INFO("[Bumper]: Depth camera horizontal FOV: %.1fdeg", horizontal_fov/M_PI*180.0);
        ROS_INFO("[Bumper]: Depth camera vertical FOV: %.1fdeg", vectical_fov/M_PI*180.0);
        ROS_INFO("[Bumper]: Number of horizontal sectors: %d", m_n_horizontal_sectors);
        m_sectors_initialized = true;
      }
      //}

      if (m_sectors_initialized)
      {
        /* Prepare the ObstacleSectors message to be published //{ */
        mrs_bumper::ObstacleSectors obst_msg;
        obst_msg.header.frame_id = m_frame_id;
        obst_msg.header.stamp = ros::Time::now();
        obst_msg.n_horizontal_sectors = m_n_horizontal_sectors;
        obst_msg.sectors_vertical_fov = m_vertical_fov;
        obst_msg.sectors.resize(m_n_total_sectors, mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN);
        obst_msg.sector_sensors.resize(m_n_total_sectors, mrs_bumper::ObstacleSectors::SENSOR_NONE);
        //}

        /* Check data from the front-facing realsense //{ */
        if (m_depthmap_sh->new_data() && m_depth_cinfo_sh->has_data())
        {
          cv_bridge::CvImage source_msg = *cv_bridge::toCvCopy(m_depthmap_sh->get_data(), std::string("16UC1"));
        
          /* Apply ROI //{ */
          cv::Rect roi(m_roi.x_offset, m_roi.y_offset, m_roi.width, m_roi.height);
          source_msg.image = source_msg.image(roi);
          //}
        
          /* Prepare the image for obstacle detection //{ */
          // create the detection image
          cv::Mat detect_im = source_msg.image.clone();
          cv::Mat raw_im = source_msg.image;
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
        
          //TODO: filter out ground?
        
          //TODO: more sophisticated detection of obstacles?
        
          cv::Mat usable_pixels;
          if (m_mask_im.empty())
            usable_pixels = known_pixels;
          else
            cv::bitwise_and(known_pixels, m_mask_im, usable_pixels);
          double min_val = m_unknown_pixel_value;
          cv::minMaxLoc(detect_im, &min_val, nullptr, nullptr, nullptr, usable_pixels);
          double obstacle_depth = min_val/1000.0;
          const bool obstacle_sure = !(min_val == m_unknown_pixel_value);
          auto& cur_value = obst_msg.sectors.at(0);
          auto& cur_sensor = obst_msg.sector_sensors.at(0);
          if (obstacle_sure && (cur_value == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN || obstacle_depth < cur_value))
          {
            cur_value = obstacle_depth;
            cur_sensor = mrs_bumper::ObstacleSectors::SENSOR_DEPTH;
          }
          if (obst_msg.header.stamp > source_msg.header.stamp)
            obst_msg.header.stamp = source_msg.header.stamp;

          if (m_processed_depthmap_pub.getNumSubscribers() > 0)
          {
            /* Create and publish the debug image //{ */
            cv_bridge::CvImage processed_depthmap_cvb = source_msg;
            processed_depthmap_cvb.image = detect_im;
            sensor_msgs::ImageConstPtr out_msg = processed_depthmap_cvb.toImageMsg();
            m_processed_depthmap_pub.publish(out_msg);
            //}
          }
        }
        //}

        /* Check data from the horizontal 2D lidar //{ */
        if (m_lidar_2d_sh->new_data())
        {
          sensor_msgs::LaserScan source_msg = *m_lidar_2d_sh->get_data();
        
          std::vector<double> obstacle_distances = find_obstacles_in_horizontal_sectors(source_msg);
          for (unsigned sector_it = 0; sector_it < m_n_horizontal_sectors; sector_it++)
          {
            const double obstacle_dist = obstacle_distances.at(sector_it);
            if (obstacle_dist == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN)
              continue;
            auto& cur_value = obst_msg.sectors.at(sector_it);
            auto& cur_sensor = obst_msg.sector_sensors.at(sector_it);
            if (cur_value == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN || obstacle_dist < cur_value)
            {
              cur_value = obstacle_dist;
              cur_sensor = mrs_bumper::ObstacleSectors::SENSOR_LIDAR_2D;
            }
            if (obst_msg.header.stamp > source_msg.header.stamp)
              obst_msg.header.stamp = source_msg.header.stamp;
          }
        }
        //}

        /* Check data from the down-facing lidar //{ */
        if (m_lidar_1d_down_sh->new_data())
        {
          sensor_msgs::Range source_msg = *m_lidar_1d_down_sh->get_data();
          const bool obstacle_sure = source_msg.range > source_msg.min_range && source_msg.range < source_msg.max_range;
          auto& cur_value = obst_msg.sectors.at(m_bottom_sector_idx);
          auto& cur_sensor = obst_msg.sector_sensors.at(m_bottom_sector_idx);
          if (obstacle_sure && (cur_value == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN || source_msg.range < cur_value))
          {
            cur_value = source_msg.range;
            cur_sensor = mrs_bumper::ObstacleSectors::SENSOR_LIDAR_1D;
          }
          if (obst_msg.header.stamp > source_msg.header.stamp)
            obst_msg.header.stamp = source_msg.header.stamp;
        }
        //}

        /* Check data from the up-facing lidar //{ */
        if (m_lidar_1d_up_sh->new_data())
        {
          sensor_msgs::Range source_msg = *m_lidar_1d_up_sh->get_data();
          const bool obstacle_sure = source_msg.range > source_msg.min_range && source_msg.range < source_msg.max_range;
          auto& cur_value = obst_msg.sectors.at(m_top_sector_idx);
          auto& cur_sensor = obst_msg.sector_sensors.at(m_top_sector_idx);
          if (obstacle_sure && (cur_value == mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN || source_msg.range < cur_value))
          {
            cur_value = source_msg.range;
            cur_sensor = mrs_bumper::ObstacleSectors::SENSOR_LIDAR_1D;
          }
          if (obst_msg.header.stamp > source_msg.header.stamp)
            obst_msg.header.stamp = source_msg.header.stamp;
        }
        //}

        /* Publish the ObstacleSectors message //{ */
        m_obstacles_pub.publish(obst_msg);
        //}
      }
    }
    //}

  private:

    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */
    int m_unknown_pixel_value;
    std::string m_frame_id;
    sensor_msgs::RegionOfInterest m_roi;
    bool m_roi_centering;
    //}

    /* ROS related variables (subscribers, timers etc.) //{ */
    std::unique_ptr<drmgr_t> m_drmgr_ptr;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::ImageConstPtr> m_depthmap_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::CameraInfoConstPtr> m_depth_cinfo_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::LaserScanConstPtr> m_lidar_2d_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::RangeConstPtr> m_lidar_1d_down_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::RangeConstPtr> m_lidar_1d_up_sh;
    ros::Publisher m_obstacles_pub;
    ros::Publisher m_processed_depthmap_pub;
    ros::Timer m_main_loop_timer;
    std::string m_node_name;
    //}

  private:

    // --------------------------------------------------------------
    // |                   Other member variables                   |
    // --------------------------------------------------------------

    /* Image mask //{ */
    cv::Mat m_mask_im;
    //}

    uint32_t m_n_horizontal_sectors;
    uint32_t m_bottom_sector_idx;
    uint32_t m_top_sector_idx;
    using angle_range_t = std::pair<double, double>;
    std::vector<angle_range_t> m_horizontal_sector_ranges;
    uint32_t m_n_total_sectors;
    uint32_t m_vertical_fov;
    bool m_sectors_initialized;

  private:

    // --------------------------------------------------------------
    // |                       Helper methods                       |
    // --------------------------------------------------------------

    /* get_horizontal_sector_angle_interval() method //{ */
    angle_range_t get_horizontal_sector_angle_interval(unsigned sector_it)
    {
      assert(sector_it < m_n_horizontal_sectors);
      const double angle_step = 2.0*M_PI/m_n_horizontal_sectors;
      const double angle_start = sector_it*angle_step - angle_step/2.0;
      const double angle_end = angle_start+angle_step + angle_step/2.0;
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
      return angle > angle_range.first && angle < angle_range.second;
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
          const double ray_angle = scan_msg.angle_min + ray_it*scan_msg.angle_increment;
          if (ray_range < min_range && angle_in_range(ray_angle, cur_angle_range))
            min_range = ray_range;
        }
        if (min_range < scan_msg.range_min || min_range > scan_msg.range_max)
          min_range = mrs_bumper::ObstacleSectors::OBSTACLE_UNKNOWN;
        ret.push_back(min_range);
      }
      return ret;
    }
    //}

  }; // class Bumper
}; // namespace mrs_bumper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_bumper::Bumper, nodelet::Nodelet)

