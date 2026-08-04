// Camera B(얼굴/gaze 전용) 드라이버 — Kinect v1(Xbox 360, Xbox NUI Camera)를 libfreenect의
// 동기(sync) API로 감싸서 RGB + registered depth(RGB에 정렬된 mm 단위 깊이)를 발행한다.
// docs/dual_camera_design.md의 Camera B 역할. gaze_bridge_node.py가 이 토픽들을 구독하도록
// 바꿔야 한다(아직 안 함 — /camera/camera/... 구독 중인 걸 /kinect/... 로 변경 필요).
//
// 공식 ROS2 Kinect v1 패키지가 없어서(freenect_stack은 ROS1 전용, openni2는 Kinect v1
// 미지원) ocams_ros2와 같은 방식으로 새로 작성. freenect_sync_* API는 콜백/스레드 관리를
// libfreenect 내부에서 알아서 해줘서, 여기서는 타이머로 폴링만 하면 된다.
//
// 카메라 인트린식(fx/fy/cx/cy)은 Kinect v1 RGB 카메라의 널리 알려진 근사값이다 — 이 특정
// 유닛을 실측 캘리브레이션한 게 아니다. 정확도가 중요해지면(예: 시선 픽셀 depth 융합)
// 체커보드로 실측 필요.

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

extern "C" {
#include <libfreenect.h>
#include <libfreenect_sync.h>
}

using namespace std::chrono_literals;

namespace
{
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int DEVICE_INDEX = 0;

// Kinect v1 RGB 카메라 근사 인트린식 (실측 아님 — 파일 상단 주석 참고)
constexpr double APPROX_FX = 525.0;
constexpr double APPROX_FY = 525.0;
constexpr double APPROX_CX = 319.5;
constexpr double APPROX_CY = 239.5;
}  // namespace

class KinectFaceNode : public rclcpp::Node
{
public:
  KinectFaceNode()
  : Node("kinect_face_node")
  {
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("kinect/rgb/image_raw", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("kinect/depth/image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("kinect/rgb/camera_info", 10);

    camera_info_ = buildCameraInfo();

    // 켜져 있다는 걸 눈으로 바로 확인할 수 있게 (freenect-camtest도 이렇게 함)
    freenect_sync_set_led(LED_GREEN, DEVICE_INDEX);

    timer_ = create_wall_timer(33ms, std::bind(&KinectFaceNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "kinect_face_node 시작 — kinect/rgb, kinect/depth 발행");
  }

  ~KinectFaceNode() override
  {
    freenect_sync_set_led(LED_OFF, DEVICE_INDEX);
    freenect_sync_stop();
  }

private:
  static sensor_msgs::msg::CameraInfo buildCameraInfo()
  {
    sensor_msgs::msg::CameraInfo info;
    info.width = WIDTH;
    info.height = HEIGHT;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {APPROX_FX, 0.0, APPROX_CX,
              0.0, APPROX_FY, APPROX_CY,
              0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {APPROX_FX, 0.0, APPROX_CX, 0.0,
              0.0, APPROX_FY, APPROX_CY, 0.0,
              0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void onTimer()
  {
    const auto stamp = now();

    void * video_buf = nullptr;
    uint32_t video_ts = 0;
    if (freenect_sync_get_video(&video_buf, &video_ts, DEVICE_INDEX, FREENECT_VIDEO_RGB) == 0) {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp;
      msg->header.frame_id = "kinect_rgb_optical_frame";
      msg->height = HEIGHT;
      msg->width = WIDTH;
      msg->encoding = "rgb8";
      msg->is_bigendian = false;
      msg->step = WIDTH * 3;
      const auto * bytes = static_cast<const uint8_t *>(video_buf);
      msg->data.assign(bytes, bytes + static_cast<size_t>(WIDTH) * HEIGHT * 3);
      rgb_pub_->publish(std::move(msg));
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RGB 프레임 못 받음");
    }

    void * depth_buf = nullptr;
    uint32_t depth_ts = 0;
    if (freenect_sync_get_depth(&depth_buf, &depth_ts, DEVICE_INDEX,
        FREENECT_DEPTH_REGISTERED) == 0)
    {
      // FREENECT_DEPTH_REGISTERED: uint16_t/px, mm 단위, RGB 프레임에 이미 정렬됨
      // (gaze_bridge_node의 depth 룩업이 그대로 재사용 가능 — RealSense aligned_depth_to_color와
      // 동일한 의미)
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp;
      msg->header.frame_id = "kinect_rgb_optical_frame";
      msg->height = HEIGHT;
      msg->width = WIDTH;
      msg->encoding = "16UC1";
      msg->is_bigendian = false;
      msg->step = WIDTH * 2;
      const auto * bytes = static_cast<const uint8_t *>(depth_buf);
      msg->data.assign(bytes, bytes + static_cast<size_t>(WIDTH) * HEIGHT * 2);
      depth_pub_->publish(std::move(msg));
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Depth 프레임 못 받음");
    }

    auto info = camera_info_;
    info.header.stamp = stamp;
    info.header.frame_id = "kinect_rgb_optical_frame";
    info_pub_->publish(info);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectFaceNode>());
  rclcpp::shutdown();
  return 0;
}
