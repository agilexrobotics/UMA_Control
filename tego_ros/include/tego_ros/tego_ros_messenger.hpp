// tego_ros/include/tego_ros/tego_ros_messenger.hpp

#ifndef TEGO_ROS_MESSENGER_HPP
#define TEGO_ROS_MESSENGER_HPP

#define RCLCPP_INFO_ONCE(logger, ...) \
  do { \
    static bool _rclcpp_info_once_flag = false; \
    if (!_rclcpp_info_once_flag) { \
      RCLCPP_INFO(logger, __VA_ARGS__); \
      _rclcpp_info_once_flag = true; \
    } \
  } while (0)

#include <string>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <mutex>
#include <map>
#include <functional>
#include <thread>
#include <memory>
#include <atomic>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "tego_sdk/base.hpp"

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "tego_msgs/msg/mt_sensor_status.hpp"
#include "tego_msgs/msg/handle_state.hpp"
#include "tego_msgs/srv/reset_control.hpp"
#include "tego_msgs/srv/change_encoder_id.hpp"
#include "tego_msgs/srv/set_baud_rate.hpp"
#include "tego_msgs/srv/set_encoder_mode.hpp"
#include "tego_msgs/srv/set_encoder_return_rate.hpp"
#include "tego_msgs/srv/set_encoder_zero.hpp"

using namespace std;

static map<bool, string> Bool2String
{
  {true, "true"},
  {false, "false"}
};

namespace tegocore
{
class TegoRosMessenger : public rclcpp::Node
{
public:
    explicit TegoRosMessenger(TegoBase *tego, const std::string& node_name = "tego_ros_node");
    ~TegoRosMessenger() override;

    void SetupSubscription();
    void PublishStateToROS();
    template<typename T>
    void GetRosParameter(const std::string &name, T &value, bool is_cfg = false)
    {
        if (!this->has_parameter(name)) {
            this->declare_parameter<T>(name, value);
        }
        this->get_parameter(name, value);
        if (!is_cfg) {
            // 测试无效
            // this->undeclare_parameter(name);
        }
    }

private:
    TegoBase *tego_;

    bool check_get_ = true;
    bool wait_feedback_ = true;
    bool publish_sim_joint_ = true;
    double gripper_width_min = -0.01;
    double gripper_width_max = 0.07;
    std::string remote_joint_topic_in = "joint_in";
    std::string remote_joint_topic_out = "joint_out";
    std::string remote_service_name = "teleop_piper_status";
    std::string node_name = "tego_node";
    string arm_home_state;
    string imu_zero_rpy = "0, 0, 0";
    string effector_type = "none";
    string nero_gripper_topic_name = "gripper_states";

    double angle_value = 0.35;
    bool angle_reverse = false;
    bool add_angle = true;
    bool base_reverse = false;
    bool imu_reverse = true;
    bool show_imu_data = true;
    bool use_piper_mode = true;
    bool is_nero = false;
    double zoom_factor = 1.0;
    std::vector<double> handle_seg0{0.0, 0.0, 0.123, 0.0, 0.0, 0.0};
    std::vector<double> handle_seg1{0.0, 0.0, 0.0, 1.5707963, -0.1359, 3.1415926};
    std::vector<double> handle_seg2{0.28503, 0.0, 0.0, 0.0, 0.0, -1.7939};
    std::vector<double> handle_seg3{-0.022, -0.25075, 0.0, 1.5707963, 0.0, 0.0};
    double base_to_imu_roll = 0.0;
    double base_to_imu_pitch = 0.0;
    double base_to_imu_yaw = 0.0;
    double world_to_base_y = 0.0;
    vector<double> arm_home_state_vec, imu_zero_rpy_vec;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr remote_joint_publisher_;
    rclcpp::Publisher<tego_msgs::msg::MtSensorStatus>::SharedPtr sensor_status_publisher_;
    rclcpp::Publisher<tego_msgs::msg::HandleState>::SharedPtr handle_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tego_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr nero_gripper_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr remote_joint_subscriber_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teleop_enable_subsciber_;

    // Service Clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr teleop_piper_cli_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_cli_;

    // Service Servers
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_pose_server_;
    rclcpp::Service<tego_msgs::srv::ResetControl>::SharedPtr reset_control_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_handle_motor_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_reverse_server_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Data buffers
    int mt_sensor_send[7] = {0};
    int handle_status_send = 0;
    int handle_calibration_send = 0;
    int key1_send = 0;
    int key2_send = 0;
    int key_event_send = 0;
    int reset_feedback_send[7] = {0};
    int imu_linear_acceleration_send = 0;
    int imu_angular_velocity_send = 0;
    int imu_orientation_send = 0;

    Eigen::Quaterniond imu2baseRPYQ;

    bool update_imu = true, update_joint = true, update_base_pose = false;
    Eigen::Quaterniond imu_base_quatd, imu_diff_quatd, imu_zero_quatd;
    bool get_imu = false;
    double angle0_base = 0, angle1_base = 0, angle2_base = 0;
    bool use_trigger_pose = false;
    double gripper_length = 0.15;

    double angle_list[7] = {0};
    int trigger_press_percentage = 0;

    std::atomic_bool is_trigger_action{false};
    std::atomic_bool trigger_action_tmp{false};
    std::atomic_bool is_init_action{false};
    std::atomic_bool init_action_tmp{false};
    std::atomic_bool teleop_enable{false};
    rclcpp::Time last_get_teleop_enable_time, last_call_teleop_time;
    sensor_msgs::msg::JointState last_arm_joint, last_remote_joint;
    std::mutex arm_joint_mutex;
    std::mutex remote_joint_publish_mutex;

    bool use_pose_dt_zoom = true;
    Eigen::Matrix4d remote_base_pose_matrix, cur_pose_matrix;

    int low_press_level = 10, high_press_level = 70;
    bool wait_press = true;
    double double_press_timeout = 0.6;
    double long_press_timeout = 2.0;
    rclcpp::Time last_press_action_time;
    int press_action_num = 0;

    std::mutex config_cli_mutex, update_mutex;
    std::mutex teleop_state_mutex;
    std::atomic_bool is_reconfig_run{false};

    // Threads started by callbacks must not outlive this node.
    std::mutex thread_mutex_;
    std::vector<std::thread> threads_;
    std::atomic_bool stopping_threads_{false};

    inline double GetGripperWidth()
    {
        return max(0.0, (gripper_width_max - gripper_width_min) / 100. * (100 - trigger_press_percentage) + gripper_width_min);
    }

    inline void makeThread(std::function<void()> fun)
    {
        if (!fun)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(thread_mutex_);
        if (stopping_threads_)
        {
            return;
        }
        threads_.emplace_back(std::move(fun));
    }

    template<typename T>
    inline void RosDurationSleep(T seconds)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(int(seconds*1000)));
    }

    const std::unordered_map<
        std::string,
        std::function<void(TegoRosMessenger&, const rclcpp::Parameter&)>
    > param_handlers = {
        {"add_angle", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.add_angle = p.as_bool();
        }},
        {"base_reverse", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.base_reverse = p.as_bool();
        }},
        {"imu_reverse", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.imu_reverse = p.as_bool();
        }},
        {"show_imu_data", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.show_imu_data = p.as_bool();
        }},
        {"use_piper_mode", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.use_piper_mode = p.as_bool();
        }},
        {"handle_seg0", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.handle_seg0 = p.as_double_array();
        }},
        {"handle_seg1", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.handle_seg1 = p.as_double_array();
        }},
        {"handle_seg2", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.handle_seg2 = p.as_double_array();
        }},
        {"handle_seg3", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.handle_seg3 = p.as_double_array();
        }},
        {"use_pose_dt_zoom", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.use_pose_dt_zoom = p.as_bool();
        }},
        {"zoom_factor", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.zoom_factor = p.as_double();
        }},
        {"use_trigger_pose", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.use_trigger_pose = p.as_bool();
        }},
        {"gripper_length", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.gripper_length = p.as_double();
        }},
        {"trigger_action", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.trigger_action_tmp.store(p.as_bool());
        }},
        {"init_action", [](TegoRosMessenger& self, const rclcpp::Parameter& p) {
            self.init_action_tmp.store(p.as_bool());
        }}
    };

    void declareParameters();
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void ResetPoseServer(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void ResetControlServer(
        const std::shared_ptr<tego_msgs::srv::ResetControl::Request> request,
        std::shared_ptr<tego_msgs::srv::ResetControl::Response> response);

    void EnableHandleMotorServer(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void EnableReverseServer(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void TeleopEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void RemoteJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ArmJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void ImuConverter(const sensor_msgs::msg::Imu::SharedPtr input);
    Eigen::Quaterniond GetImuQuaterniond(const sensor_msgs::msg::Imu& imu);
    Eigen::Quaterniond GetRPYQuaterniond(double roll, double pitch, double yaw);
    Eigen::Matrix4d ToTransform(double x=0.0, double y=0.0, double z=0.0,
                                double roll=0.0, double pitch=0.0, double yaw=0.0, double zoom_factor=1.0);
    Eigen::Matrix4d ToTransform(double x, double y, double z, Eigen::Quaterniond q, double zoom_factor=1.0);
    void UpdateAndPublishPose();
    void PublishRemotePose();
    void PublishTF(const Eigen::Matrix4d& transform, const std::string& frame_id, const std::string& child_frame_id);
    void AnalyzeKeyEventAction(uint8_t key_id, uint8_t click_type);
    bool CheckTimeout(rclcpp::Time time, double timeout = 1.0);
    bool CheckTriggerStatus(bool cur_trigger_status);
    void SplitRosParams(const std::string& data, std::vector<double>& vec);
    void SetHandleMotor(bool run = false);
    void SetHandlelight(HandleLightControlCmd::ColorId color);
    void ResetTriggerActionThread();
    void ResetTeleopEnableThread();
};

} // namespace tegocore

#endif /* TEGO_ROS_MESSENGER_HPP */
