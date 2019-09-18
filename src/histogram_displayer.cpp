// clang: MatousFormat
#include <ros/ros.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mrs_msgs/Histogram.h>

cv::Point cursor_pos;
void mouse_callback([[maybe_unused]] int event, int x, int y, [[maybe_unused]] int flags, [[maybe_unused]] void* userdata)
{
  cursor_pos = cv::Point(x, y);
}

/* draw_hist() function //{ */
void draw_hist(const std::vector<float>& hist, cv::Mat& hist_img, unsigned highlight_first = 0)
{
  double max_val = 0;
  const auto bins = hist.size();
  cv::minMaxLoc(hist, 0, &max_val, 0, 0);
  const unsigned height = hist_img.rows;
  const float hscale = height / max_val;
  const float vscale = hist_img.cols / bins;
  for (unsigned b = 0; b < bins; b++)
  {
    const auto bin_val = hist.at(b);
    cv::Scalar color = cv::Scalar::all(255);
    if (b < highlight_first)
      color = cv::Scalar(255, 0, 0);
    cv::rectangle(hist_img, cv::Point(b * vscale, height), cv::Point(b * vscale + vscale, height - bin_val * hscale), color, -1);
  }
}
//}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "histogram_displayer");
  ROS_INFO("Node initialized.");

  ros::NodeHandle nh = ros::NodeHandle("~");

  /** Create publishers and subscribers //{**/
  // Initialize other subs and pubs
  mrs_lib::SubscribeMgr smgr(nh);
  const bool subs_time_consistent = false;
  auto sh_hist = smgr.create_handler<mrs_msgs::Histogram, subs_time_consistent>("histogram", ros::Duration(5.0));

  //}

  /* Open OpenCV windows to display the debug info //{ */
  int window_flags = cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL;
  std::string winname = "histogram";
  cv::namedWindow(winname, window_flags);
  cv::setMouseCallback(winname, mouse_callback, NULL);
  //}

  ros::Rate r(30);
  const int bot_rows = 100;
  const int right_cols = 100;

  while (ros::ok())
  {
    ros::spinOnce();

    if (sh_hist->new_data())
    {
      const mrs_msgs::Histogram hist_msg = *(sh_hist->get_data());
      const int hr = 1000;
      const int hc = 1000;
      cv::Mat disp_im = cv::Mat::zeros(hr + bot_rows, hc + right_cols, CV_8UC3);
      cv::Mat hist_roi = disp_im(cv::Rect(cv::Point(0, 0), cv::Size(hr, hc)));
      draw_hist(hist_msg.bins, hist_roi, hist_msg.bin_mark);
      /* hist_roi(cv::Rect(cv::Point(vscale*hist_msg.bin_mark, 0), cv::Point(vscale*hist_msg.bin_mark+1, hc-1))) = cv::Scalar(255, 0, 0); */
      cv::imshow(winname, disp_im);
      cv::waitKey(1);
    }

    r.sleep();
  }
}

