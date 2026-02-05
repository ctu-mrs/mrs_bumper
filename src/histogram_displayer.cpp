// clang: MatousFormat
#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/node.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mrs_msgs/msg/histogram.hpp>

namespace mrs_bumper
{

  class HistogramDisplayer : public mrs_lib::Node
  {
  public:
    HistogramDisplayer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : mrs_lib::Node("histogram_displayer", options)
    {
      RCLCPP_INFO(this_node_ptr()->get_logger(), "Node initialized.");

      int window_flags = cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL;
      winname_ = "histogram";
      cv::namedWindow(winname_, window_flags);
      cv::setMouseCallback(winname_, mouse_callback, NULL);

      mrs_lib::SubscriberHandlerOptions shopts(this_node_ptr());
      sh_hist = mrs_lib::SubscriberHandler<mrs_msgs::msg::Histogram>(shopts, "~/histogram", rclcpp::Duration::from_seconds(5.0));

      timer_ = this_node_ptr()->create_wall_timer(std::chrono::milliseconds(33), [this]() {
        if (sh_hist.newMsg())
        {
          const mrs_msgs::msg::Histogram hist_msg = *(sh_hist.getMsg());
          const int hr = 1000;
          const int hc = 1000;
          const int bot_rows = 100;
          const int right_cols = 100;
          cv::Mat disp_im = cv::Mat::zeros(hr + bot_rows, hc + right_cols, CV_8UC3);
          cv::Mat hist_roi = disp_im(cv::Rect(cv::Point(0, 0), cv::Size(hr, hc)));
          draw_hist(hist_msg.bins, hist_roi, hist_msg.bin_mark);
          cv::imshow(winname_, disp_im);
          cv::waitKey(1);
        }
      });
    }

  private:
    mrs_lib::SubscriberHandler<mrs_msgs::msg::Histogram> sh_hist;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string winname_;

    static cv::Point cursor_pos;
    static void mouse_callback([[maybe_unused]] int event, int x, int y, [[maybe_unused]] int flags, [[maybe_unused]] void* userdata)
    {
      cursor_pos = cv::Point(x, y);
    }

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
  };

  cv::Point HistogramDisplayer::cursor_pos = cv::Point(0, 0);

} // namespace mrs_bumper

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_bumper::HistogramDisplayer)
