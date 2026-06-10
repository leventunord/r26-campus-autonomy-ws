#include "fisheye_stitcher.hpp"
#include "input_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class FisheyeStitcherNode : public rclcpp::Node {
public:
    FisheyeStitcherNode()
        : Node("fisheye_stitcher_node")
    {
        // 声明参数：MLS 地图路径（默认从包安装目录解析）
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("fisheye_process");
        std::string default_map_path = pkg_dir + "/utils/grid_xd_yd_3840x1920.yml.gz";
        this->declare_parameter("mls_map_path", default_map_path);
        std::string mls_map_path = this->get_parameter("mls_map_path").as_string();

        RCLCPP_INFO(this->get_logger(), "MLS map path: %s", mls_map_path.c_str());

        Stitcher_ = std::make_unique<stitcher::FisheyeStitcher>(
            3840, 1920, 210.0f, true, true, mls_map_path);
    }

    void init() {
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        sub_ = it_->subscribe("/image_raw", 1,
            std::bind(&FisheyeStitcherNode::imageCallback, this, std::placeholders::_1));
        pub_ = it_->advertise("/image_stitched", 1);
        RCLCPP_INFO(this->get_logger(), "Fisheye stitcher node initialized: /image_raw -> /image_stitched");
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;
        cv::Mat initMat;
        cv::resize(img, initMat, cv::Size(3840, 1920), 0, 0, cv::INTER_LINEAR);
        cv::Mat img_l = initMat(cv::Rect(0, 0, initMat.cols / 2, initMat.rows));
        cv::Mat img_r = initMat(cv::Rect(initMat.cols / 2, 0, initMat.cols / 2, initMat.rows));
        cv::Mat pano = Stitcher_->stitch(img_l, img_r);

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = pano;
        pub_.publish(out_msg.toImageMsg());
    }

private:
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    std::unique_ptr<stitcher::FisheyeStitcher> Stitcher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FisheyeStitcherNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
