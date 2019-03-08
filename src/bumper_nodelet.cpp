#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>

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
      pl.load_param("unknown_pixel_value", m_unknown_pixel_value);
      pl.load_param("max_obstacle_depth", m_max_obstacle_depth);
      pl.load_param("unknown_obstacle_value", m_unknown_obstacle_value);
      pl.load_param("frame_id", m_frame_id);
      m_roi.x_offset = pl.load_param2<int>("roi/x_offset", 0);
      m_roi.y_offset = pl.load_param2<int>("roi/y_offset", 0);
      m_roi.width = pl.load_param2<int>("roi/width", 0);
      m_roi.height = pl.load_param2<int>("roi/height", 0);
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
      // Initialize publishers
      m_obstacles_pub = nh.advertise<mrs_bumper::ObstacleSectors>("obstacle_sectors", 1);
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
      //}

      m_main_loop_timer = nh.createTimer(ros::Rate(1000), &Bumper::main_loop, this);

      std::cout << "----------------------------------------------------------" << std::endl;

    }
    //}

    /* main_loop() method //{ */
    void main_loop([[maybe_unused]] const ros::TimerEvent& evt)
    {
      /* Update number of horizontal sectors etd //{ */
      if (m_depth_cinfo_sh->new_data())
      {
        const double fx = m_depth_cinfo_sh->get_data()->K[0];
        const double cx = m_depth_cinfo_sh->get_data()->K[2];
        const double horizontal_fov = std::atan2(cx, fx)*2.0;
        m_n_horizontal_sectors = std::ceil(2.0*M_PI/horizontal_fov);
        /* std::cout << "horizontal FOV: " << horizontal_fov << std::endl; */
        /* std::cout << "horizontal sectors: " << m_n_horizontal_sectors << std::endl; */
        m_n_total_sectors = m_n_horizontal_sectors + 2;
      }
      //}

      if (m_depthmap_sh->new_data())
      {
        cv_bridge::CvImage source_msg = *cv_bridge::toCvCopy(m_depthmap_sh->get_data(), std::string("16UC1"));

        /* Apply ROI //{ */
        if (m_roi.y_offset + m_roi.height > unsigned(source_msg.image.rows) || m_roi.height == 0)
          m_roi.height = std::clamp(int(source_msg.image.rows - m_roi.y_offset), 0, source_msg.image.rows);
        if (m_roi.x_offset + m_roi.width > unsigned(source_msg.image.cols) || m_roi.width == 0)
          m_roi.width = std::clamp(int(source_msg.image.cols- m_roi.x_offset), 0, source_msg.image.cols);
        
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

        //TODO: filter out ground
        
        //TODO: detection of obstacles
        
        cv::Mat usable_pixels;
        if (m_mask_im.empty())
          usable_pixels = known_pixels;
        else
          cv::bitwise_and(known_pixels, m_mask_im, usable_pixels);
        double min_val = m_unknown_pixel_value;
        cv::minMaxLoc(detect_im, &min_val, nullptr, nullptr, nullptr, usable_pixels);
        double obstacle_depth = min_val/1000;
        std::cout << "min. value: " << min_val << std::endl;
        if (min_val == m_unknown_pixel_value || obstacle_depth > m_max_obstacle_depth)
          obstacle_depth = m_unknown_obstacle_value;

        mrs_bumper::ObstacleSectors obst_msg;
        obst_msg.header.frame_id = m_frame_id;
        obst_msg.header.stamp = source_msg.header.stamp;
        obst_msg.n_horizontal_sectors = m_n_horizontal_sectors;
        obst_msg.sectors.resize(m_n_total_sectors, m_unknown_obstacle_value);
        obst_msg.sectors.at(0) = obstacle_depth;
        m_obstacles_pub.publish(obst_msg);
      }
    }
    //}

  private:

    // --------------------------------------------------------------
    // |                ROS-related member variables                |
    // --------------------------------------------------------------

    /* Parameters, loaded from ROS //{ */
    int m_unknown_pixel_value;
    double m_unknown_obstacle_value;
    double m_max_obstacle_depth;
    std::string m_frame_id;
    sensor_msgs::RegionOfInterest m_roi;
    //}

    /* ROS related variables (subscribers, timers etc.) //{ */
    std::unique_ptr<drmgr_t> m_drmgr_ptr;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::ImageConstPtr> m_depthmap_sh;
    mrs_lib::SubscribeHandlerPtr<sensor_msgs::CameraInfoConstPtr> m_depth_cinfo_sh;
    ros::Publisher m_obstacles_pub;
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
    uint32_t m_n_total_sectors;

  }; // class Bumper
}; // namespace mrs_bumper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_bumper::Bumper, nodelet::Nodelet)

