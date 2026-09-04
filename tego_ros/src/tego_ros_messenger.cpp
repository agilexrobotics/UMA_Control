#include "tego_ros/tego_ros_messenger.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <set>

using namespace std::placeholders;

namespace tegocore
{

TegoRosMessenger::TegoRosMessenger(TegoBase *tego, const std::string& node_name):
    rclcpp::Node(node_name),
    tego_(tego),
    arm_home_state_vec(),
    imu_zero_rpy_vec(vector<double>(3, 0.0))
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

TegoRosMessenger::~TegoRosMessenger()
{
    {
        std::lock_guard<std::mutex> lock(thread_mutex_);
        stopping_threads_.store(true);
    }

    for (auto &thread : threads_)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
}

void TegoRosMessenger::SetupSubscription()
{
    // Parameter
    declareParameters();

    // Publisher
    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint", 200);
    if (publish_sim_joint_)
    {
        sim_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_single", 200);
    }
    sensor_status_publisher_ = this->create_publisher<tego_msgs::msg::MtSensorStatus>("~/mt_sensor_status", 200);
    handle_state_publisher_ = this->create_publisher<tego_msgs::msg::HandleState>("~/handle_state", 200);
    tego_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pika_pose", 200);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", 200);
    remote_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(remote_joint_topic_out, 200);
    if (is_nero)
    {
        nero_gripper_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(nero_gripper_topic_name, 200);
    }

    // Subscriber
    remote_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(remote_joint_topic_in, 200,
        std::bind(&TegoRosMessenger::RemoteJointCallback, this, _1));
    arm_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states_single", 200,
        std::bind(&TegoRosMessenger::ArmJointCallback, this, _1));
    teleop_enable_subsciber_ = this->create_subscription<std_msgs::msg::Bool>("/teleop_enable", 200,
        std::bind(&TegoRosMessenger::TeleopEnableCallback, this, _1));

    // Server
    reset_pose_server_ = this->create_service<std_srvs::srv::Empty>(
        "~/reset_pose",
        std::bind(&TegoRosMessenger::ResetPoseServer, this, _1, _2));

    reset_control_server_ = this->create_service<tego_msgs::srv::ResetControl>(
        "~/reset_control",
        std::bind(&TegoRosMessenger::ResetControlServer, this, _1, _2));

    enable_handle_motor_server_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable_handle_motor",
        std::bind(&TegoRosMessenger::EnableHandleMotorServer, this, _1, _2));

    enable_reverse_server_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable_reverse",
        std::bind(&TegoRosMessenger::EnableReverseServer, this, _1, _2));

    // Client
    set_parameters_cli_ = this->create_client<rcl_interfaces::srv::SetParameters>("set_parameters");
    teleop_piper_cli_ = this->create_client<std_srvs::srv::Trigger>(remote_service_name);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TegoRosMessenger::dynamicParametersCallback, this, _1));

    makeThread(std::bind(&TegoRosMessenger::ResetTeleopEnableThread, this));
    makeThread((bind(&TegoRosMessenger::ResetTriggerActionThread, this)));
}

void TegoRosMessenger::declareParameters()
{
    GetRosParameter("check_get", check_get_);
    GetRosParameter("wait_feedback", wait_feedback_);
    GetRosParameter("publish_sim_joint", publish_sim_joint_);
    GetRosParameter("gripper_width_min", gripper_width_min);
    GetRosParameter("gripper_width_max", gripper_width_max);
    GetRosParameter("world_to_base_y", world_to_base_y);
    GetRosParameter("low_press_level", low_press_level);
    GetRosParameter("high_press_level", high_press_level);
    GetRosParameter("double_press_timeout", double_press_timeout);
    GetRosParameter("long_press_timeout", long_press_timeout);
    GetRosParameter("remote_joint_topic_in", remote_joint_topic_in);
    GetRosParameter("remote_joint_topic_out", remote_joint_topic_out);
    GetRosParameter("remote_service_name", remote_service_name);
    GetRosParameter("node_name", node_name);
    GetRosParameter("angle_reverse", angle_reverse);
    GetRosParameter("angle_value", angle_value);
    GetRosParameter("base_to_imu_roll", base_to_imu_roll);
    GetRosParameter("base_to_imu_pitch", base_to_imu_pitch);
    GetRosParameter("base_to_imu_yaw", base_to_imu_yaw);
    GetRosParameter("arm_home_state", arm_home_state);
    GetRosParameter("imu_zero_rpy", imu_zero_rpy);
    GetRosParameter("is_nero", is_nero);
    GetRosParameter("nero_gripper_topic_name", nero_gripper_topic_name);

    GetRosParameter("add_angle", add_angle, true);
    GetRosParameter("base_reverse", base_reverse, true);
    GetRosParameter("imu_reverse", imu_reverse, true);
    GetRosParameter("show_imu_data", show_imu_data, true);
    GetRosParameter("use_piper_mode", use_piper_mode, true);
    GetRosParameter("handle_seg0", handle_seg0, true);
    GetRosParameter("handle_seg1", handle_seg1, true);
    GetRosParameter("handle_seg2", handle_seg2, true);
    GetRosParameter("handle_seg3", handle_seg3, true);
    GetRosParameter("use_pose_dt_zoom", use_pose_dt_zoom, true);
    GetRosParameter("zoom_factor", zoom_factor, true);
    GetRosParameter("use_trigger_pose", use_trigger_pose, true);
    GetRosParameter("gripper_length", gripper_length, true);
    bool trigger_action_parameter = false;
    GetRosParameter("trigger_action", trigger_action_parameter, true);
    is_trigger_action.store(trigger_action_parameter);
    trigger_action_tmp.store(trigger_action_parameter);
    bool init_action_parameter = false;
    GetRosParameter("init_action", init_action_parameter, true);
    is_init_action.store(init_action_parameter);
    init_action_tmp.store(init_action_parameter);

    RCLCPP_INFO(this->get_logger(), "Version: [ros2] 0.1.0");
    RCLCPP_INFO(this->get_logger(), "check_get: %s", Bool2String[check_get_].c_str());
    RCLCPP_INFO(this->get_logger(), "wait_feedback: %s", Bool2String[wait_feedback_].c_str());
    RCLCPP_INFO(this->get_logger(), "publish_sim_joint: %s", Bool2String[publish_sim_joint_].c_str());
    RCLCPP_INFO(this->get_logger(), "gripper_width_min: %lf", gripper_width_min);
    RCLCPP_INFO(this->get_logger(), "gripper_width_max: %lf", gripper_width_max);
    RCLCPP_INFO(this->get_logger(), "world_to_base_y: %lf", world_to_base_y);
    RCLCPP_INFO(this->get_logger(), "low_press_level: %d", low_press_level);
    RCLCPP_INFO(this->get_logger(), "high_press_level: %d", high_press_level);
    RCLCPP_INFO(this->get_logger(), "double_press_timeout: %lf", double_press_timeout);
    RCLCPP_INFO(this->get_logger(), "long_press_timeout: %lf", long_press_timeout);
    RCLCPP_INFO(this->get_logger(), "add_angle: %s", Bool2String[add_angle].c_str());
    RCLCPP_INFO(this->get_logger(), "angle_reverse: %s", Bool2String[angle_reverse].c_str());
    RCLCPP_INFO(this->get_logger(), "base_reverse: %s", Bool2String[base_reverse].c_str());
    RCLCPP_INFO(this->get_logger(), "imu_reverse: %s", Bool2String[imu_reverse].c_str());
    RCLCPP_INFO(this->get_logger(), "show_imu_data: %s", Bool2String[show_imu_data].c_str());
    RCLCPP_INFO(this->get_logger(), "use_piper_mode: %s", Bool2String[use_piper_mode].c_str());
    RCLCPP_INFO(this->get_logger(), "use_pose_dt_zoom: %s", Bool2String[use_pose_dt_zoom].c_str());
    RCLCPP_INFO(this->get_logger(), "zoom_factor: %lf", zoom_factor);
    RCLCPP_INFO(this->get_logger(), "base_to_imu: (%-.2lf, %-.2lf, %-.2lf)",
                base_to_imu_roll, base_to_imu_pitch, base_to_imu_yaw);
    RCLCPP_INFO(this->get_logger(), "use_trigger_pose: %s", Bool2String[use_trigger_pose].c_str());
    RCLCPP_INFO(this->get_logger(), "arm_home_state: %s", arm_home_state.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_zero_rpy: %s", imu_zero_rpy.c_str());
    RCLCPP_INFO(this->get_logger(), "gripper_length: %-.2lf", gripper_length);
    RCLCPP_INFO(this->get_logger(), "is_nero: %s", Bool2String[is_nero].c_str());
    base_to_imu_roll = base_to_imu_roll / 180.0 * M_PI;
    base_to_imu_pitch = base_to_imu_pitch / 180.0 * M_PI;
    base_to_imu_yaw = base_to_imu_yaw / 180.0 * M_PI;
    try
    {
        SplitRosParams(arm_home_state, arm_home_state_vec);
        SplitRosParams(imu_zero_rpy, imu_zero_rpy_vec);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(this->get_logger(), "Invalid arm_home_state/imu_zero_rpy parameter: %s", e.what());
        throw;
    }

    imu2baseRPYQ = GetRPYQuaterniond(base_to_imu_roll, base_to_imu_pitch, base_to_imu_yaw).inverse();
    imu_diff_quatd = GetRPYQuaterniond(0, 0, 0);
    imu_zero_quatd = GetRPYQuaterniond(imu_zero_rpy_vec.at(0),
                                       imu_zero_rpy_vec.at(1),
                                       imu_zero_rpy_vec.at(2));
    remote_base_pose_matrix = ToTransform();

    auto now = this->now();
    last_get_teleop_enable_time = now;
    last_call_teleop_time = now;
    last_press_action_time = now;
}

rcl_interfaces::msg::SetParametersResult
TegoRosMessenger::dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    RCLCPP_INFO_ONCE(this->get_logger(), "get dynamic");
    is_reconfig_run.store(true);
    // get dynamic parameters
    for (const auto &param : parameters)
    {
        auto param_name = param.get_name();
        auto it = param_handlers.find(param_name);
        if (it != param_handlers.end())
        {
             it->second(*this, param);
        }
    }

    // check trigger_action
    const bool requested_trigger_action = trigger_action_tmp.load();
    const bool current_trigger_action = is_trigger_action.load();
    if (current_trigger_action != requested_trigger_action)
    {
        is_trigger_action.store(requested_trigger_action);
        bool current_teleop_enable;
        {
            std::lock_guard<std::mutex> lock(teleop_state_mutex);
            current_teleop_enable = teleop_enable.load();
            if (requested_trigger_action != current_teleop_enable)
            {
                last_call_teleop_time = this->now();
            }
        }
        if (requested_trigger_action != current_teleop_enable)
        {
            RCLCPP_INFO(this->get_logger(), "call teleop server.");
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            teleop_piper_cli_->async_send_request(req);
        }
        makeThread([this, requested_trigger_action]
        {
            if (!requested_trigger_action)
            {
                SetHandleMotor(true);
                RosDurationSleep(0.15);
                SetHandleMotor(false);
                RosDurationSleep(0.05);
                SetHandleMotor(true);
                RosDurationSleep(0.15);
                SetHandleMotor(false);
            }
            else
            {
                SetHandleMotor(true);
                RosDurationSleep(0.25);
                SetHandleMotor(false);
            }
        });
    }

    const bool init_action_requested = init_action_tmp.load();
    if (init_action_requested && teleop_enable.load())
    {
        init_action_tmp.store(false);
        result.successful = false;
        result.reason = "cannot start home action while teleop is enabled";
        is_reconfig_run.store(false);
        return result;
    }
    bool expected_not_running = false;
    if (init_action_requested &&
        is_init_action.compare_exchange_strong(expected_not_running, true))
    {
        sensor_msgs::msg::JointState arm_joint;
        {
            std::lock_guard<std::mutex> lock(arm_joint_mutex);
            arm_joint = last_arm_joint;
        }
        const bool get_arm_joint = !CheckTimeout(arm_joint.header.stamp, 3.0);

        makeThread([this, arm_joint = std::move(arm_joint), get_arm_joint]() mutable
        {
            RCLCPP_INFO(this->get_logger(), "start init action thread");
            const auto finish_init_action = [this]()
            {
                is_init_action.store(false);
                init_action_tmp.store(false);
                this->set_parameter(rclcpp::Parameter("init_action", false));
                RCLCPP_INFO(this->get_logger(), "stop init action thread");
            };

            if (!get_arm_joint)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "no fresh arm joint feedback; aborting home action");
                finish_init_action();
                return;
            }
            if (arm_joint.name.empty() ||
                arm_joint.name.size() != arm_joint.position.size() ||
                std::set<std::string>(arm_joint.name.begin(), arm_joint.name.end()).size() !=
                    arm_joint.name.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "invalid feedback JointState name/position layout; aborting home action");
                finish_init_action();
                return;
            }
            for (const auto position : arm_joint.position)
            {
                if (!std::isfinite(position))
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "feedback JointState contains non-finite position; aborting home action");
                    finish_init_action();
                    return;
                }
            }
            if (arm_home_state_vec.size() != arm_joint.position.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "arm_home_state length %zu does not match feedback joint length %zu; aborting home action",
                    arm_home_state_vec.size(), arm_joint.position.size());
                finish_init_action();
                return;
            }
            if (teleop_enable.load())
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "teleop enabled; canceling home action");
                finish_init_action();
                return;
            }

            SetHandleMotor(true);
            double init_time = 0.6;
            int init_rate = 50;
            size_t init_steps = static_cast<size_t>(init_rate * init_time);
            rclcpp::Rate loop_rate(init_rate);

            std::vector<double> angle_dt_vec(arm_joint.position.size(), 0.0);
            for (size_t i = 0; i < arm_joint.position.size(); i++)
            {
                angle_dt_vec.at(i) = (arm_joint.position.at(i) - arm_home_state_vec.at(i)) / init_steps;
            }

            std::cout << "[dt]: " << std::flush;
            for (const auto &it : angle_dt_vec)
            {
                std::cout << it << ", " << std::flush;
            }
            std::cout << std::endl;

            for (size_t i = 0; i < init_steps; ++i)
            {
                if (teleop_enable.load())
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "teleop enabled; canceling home action");
                    SetHandleMotor(false);
                    finish_init_action();
                    return;
                }
                for (size_t j = 0; j < arm_joint.position.size(); ++j)
                {
                    arm_joint.position[j] -= angle_dt_vec[j];
                }
                bool teleop_interrupted = false;
                {
                    std::lock_guard<std::mutex> lock(remote_joint_publish_mutex);
                    if (teleop_enable.load())
                    {
                        teleop_interrupted = true;
                    }
                    else
                    {
                        remote_joint_publisher_->publish(arm_joint);
                    }
                }
                if (teleop_interrupted)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "teleop enabled; canceling home action");
                    SetHandleMotor(false);
                    finish_init_action();
                    return;
                }
                loop_rate.sleep();
            }

            if (teleop_enable.load())
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "teleop enabled; canceling home action");
                SetHandleMotor(false);
                finish_init_action();
                return;
            }
            arm_joint.position = arm_home_state_vec;
            {
                std::lock_guard<std::mutex> lock(remote_joint_publish_mutex);
                if (teleop_enable.load())
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "teleop enabled; canceling home action");
                    SetHandleMotor(false);
                    finish_init_action();
                    return;
                }
                remote_joint_publisher_->publish(arm_joint);
            }
            SetHandleMotor(false);
            finish_init_action();
        });
    }
    is_reconfig_run.store(false);

    return result;
}

void TegoRosMessenger::EnableHandleMotorServer(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    if (!tego_->IsConnected())
    {
        res->success = false;
        res->message = "TEGO CAN is not connected";
        return;
    }
    SetHandleMotor(req->data);
    res->success = true;
    res->message = "success";
}

void TegoRosMessenger::ResetPoseServer(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    update_imu = true;
    get_imu = false;
    update_joint = true;
}

void TegoRosMessenger::ResetControlServer(
    const std::shared_ptr<tego_msgs::srv::ResetControl::Request> req,
    std::shared_ptr<tego_msgs::srv::ResetControl::Response> res)
{
    if (!tego_->IsConnected())
    {
        res->success = false;
        res->message = "TEGO CAN is not connected";
        return;
    }
    MtResetCmd cmd;
    cmd.reset_mode = req->reset_mode == tego_msgs::srv::ResetControl::Request::RESET_FACTORY
        ? MtResetCmd::ResetMode::FACTORY
        : MtResetCmd::ResetMode::ZERO;
    tego_->SetResetCommand(cmd);
    res->success = true;
    res->message = "reset command sent";
}

void TegoRosMessenger::EnableReverseServer(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "get reverse request: %s", Bool2String[req->data].c_str());
    if (base_reverse != req->data && get_imu)
    {
        RCLCPP_INFO(this->get_logger(), "reverse imu.");
        auto last_quatd = imu2baseRPYQ;
        imu2baseRPYQ = GetRPYQuaterniond(base_to_imu_roll, base_to_imu_pitch, base_to_imu_yaw + req->data * M_PI).inverse();
        imu_base_quatd = imu_base_quatd * last_quatd.inverse() * imu2baseRPYQ;
    }
    base_reverse = req->data;
    res->success = true;
    res->message = "success.";
}

void TegoRosMessenger::RemoteJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    last_remote_joint = *msg;
    last_remote_joint.header.stamp = this->now();

    if (!is_nero && last_remote_joint.position.size() >= 7)
    {
        last_remote_joint.position[6] = GetGripperWidth();
    }
    {
        std::lock_guard<std::mutex> lock(remote_joint_publish_mutex);
        if (teleop_enable.load())
        {
            remote_joint_publisher_->publish(last_remote_joint);
        }
    }
}

void TegoRosMessenger::ArmJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(arm_joint_mutex);
    last_arm_joint = *msg;
    last_arm_joint.header.stamp = this->now();
}

void TegoRosMessenger::TeleopEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> state_lock(teleop_state_mutex);
    std::lock_guard<std::mutex> publish_lock(remote_joint_publish_mutex);
    last_get_teleop_enable_time = this->now();
    teleop_enable.store(msg->data);
}

void TegoRosMessenger::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (show_imu_data)
    {
        auto tt = GetImuQuaterniond(*msg);
        tf2::Quaternion quat(tt.x(), tt.y(), tt.z(), tt.w());
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        cout << "(" << roll * 180. / M_PI
             << ", " << pitch * 180. / M_PI
             <<  ", " << yaw * 180. / M_PI
             << ")" << endl;
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "get imu, base_reverse: %s", Bool2String[base_reverse].c_str());

    ImuConverter(msg);
    get_imu = true;
    if (update_imu)
    {
        update_imu = false;
        imu_base_quatd = GetImuQuaterniond(*msg);
        update_base_pose = true;
    }
    auto cur_imu_quatd = GetImuQuaterniond(*msg);
    imu_diff_quatd = imu_base_quatd.inverse() * cur_imu_quatd;
    PublishTF(ToTransform(0, 0, 0, imu_diff_quatd), "imu_base", "imu_diff");
}

void TegoRosMessenger::ImuConverter(const sensor_msgs::msg::Imu::SharedPtr input)
{
    Eigen::Quaterniond q_from(GetImuQuaterniond(*input));
    Eigen::Quaterniond q_final = q_from * imu2baseRPYQ;
    input->orientation.x = q_final.x();
    input->orientation.y = q_final.y();
    input->orientation.z = q_final.z();
    input->orientation.w = q_final.w();
}

Eigen::Quaterniond TegoRosMessenger::GetImuQuaterniond(const sensor_msgs::msg::Imu &imu)
{
    return Eigen::Quaterniond(imu.orientation.w, imu.orientation.x,
                              imu.orientation.y, imu.orientation.z);
}

Eigen::Quaterniond TegoRosMessenger::GetRPYQuaterniond(double roll, double pitch, double yaw)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

Eigen::Matrix4d TegoRosMessenger::ToTransform(double x, double y, double z, double roll, double pitch, double yaw, double zoom_factor)
{
    return (Eigen::Translation3d(x*zoom_factor, y*zoom_factor, z*zoom_factor) *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).matrix();
}

Eigen::Matrix4d TegoRosMessenger::ToTransform(double x, double y, double z, Eigen::Quaterniond q, double zoom_factor)
{
    return Eigen::Matrix4d((Eigen::Translation3d(x*zoom_factor, y*zoom_factor, z*zoom_factor) * q).matrix());
}

void TegoRosMessenger::PublishTF(const Eigen::Matrix4d &transform, const std::string &frame_id, const std::string &child_frame_id)
{
    tf2::Matrix3x3 tf_rotation_matrix(
        transform(0, 0), transform(0, 1), transform(0, 2),
        transform(1, 0), transform(1, 1), transform(1, 2),
        transform(2, 0), transform(2, 1), transform(2, 2)
    );
    tf2::Quaternion q;
    tf_rotation_matrix.getRotation(q);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform.translation.x = transform(0, 3);
    tf_msg.transform.translation.y = transform(1, 3);
    tf_msg.transform.translation.z = transform(2, 3);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}

void TegoRosMessenger::UpdateAndPublishPose()
{
    double world_to_joint_base_yaw = base_reverse ? M_PI : 0;
    int angle_reverse_k = angle_reverse ? -1 : 1;
    double angle0 = -(angle_list[1] - angle0_base);// + arm_home_state_vec.at(0);
    double angle1 = angle_reverse_k * (angle_list[2] - angle1_base);// + arm_home_state_vec.at(1);
    double angle2 = -angle_reverse_k * (angle_list[3] - angle2_base) + add_angle * angle_value;// + arm_home_state_vec.at(2);
    double gripper_pos = GetGripperWidth()/2;

    auto cur_imu_diff_quatd = imu_zero_quatd * imu_diff_quatd;
    if (imu_reverse)
    {
        double diff_roll, diff_pitch, diff_yaw;
        tf2::Quaternion quat(imu_diff_quatd.x(), imu_diff_quatd.y(), imu_diff_quatd.z(), imu_diff_quatd.w());
        tf2::Matrix3x3(quat).getRPY(diff_roll, diff_pitch, diff_yaw);
        cur_imu_diff_quatd = GetRPYQuaterniond(-diff_roll, -diff_pitch, diff_yaw);
    }

    Eigen::Matrix4d joint_base_to_joint0_matrix, joint0_to_joint1_matrix, joint1_to_joint2_matrix, joint2_to_joint3_matrix;
    Eigen::Matrix4d world_to_joint_base_matrix(ToTransform(0, world_to_base_y, 0, 0, 0, world_to_joint_base_yaw));
    Eigen::Matrix4d gripper_length_matrix(ToTransform(gripper_length));
    Eigen::Matrix4d gripper_left_matrix(ToTransform(gripper_length, gripper_pos));
    Eigen::Matrix4d gripper_right_matrix(ToTransform(gripper_length, -gripper_pos, 0, M_PI));

    // 手柄运动学DH参数常量
    double seg_zoom = use_piper_mode ? 1.0 : (use_pose_dt_zoom ? 1.0 : zoom_factor);
    joint_base_to_joint0_matrix = ToTransform(handle_seg0.at(0), handle_seg0.at(1), handle_seg0.at(2),
                                              handle_seg0.at(3), handle_seg0.at(4), angle0 + handle_seg0.at(5));
    joint0_to_joint1_matrix = ToTransform(handle_seg1.at(0), handle_seg1.at(1), handle_seg1.at(2),
                                          handle_seg1.at(3), handle_seg1.at(4) - angle1, handle_seg1.at(5));
    joint1_to_joint2_matrix = ToTransform(handle_seg2.at(0), handle_seg2.at(1), handle_seg2.at(2),
                                          handle_seg2.at(3), handle_seg2.at(4), angle2 + handle_seg2.at(5),
                                          seg_zoom);
    joint2_to_joint3_matrix = ToTransform(handle_seg3.at(0), handle_seg3.at(1), handle_seg3.at(2) * angle_reverse_k,
                                          handle_seg3.at(3), handle_seg3.at(4), handle_seg3.at(5),
                                          seg_zoom);

    Eigen::Matrix4d joint3_to_joint4_matrix(ToTransform(-0.00424, 0, 0.03474));
    Eigen::Matrix4d joint4_to_joint5_matrix(ToTransform());
    if (use_trigger_pose)
    {
        // joint3 -> joint4 x: -0.0264->-0.00424  z: 0.02235->0.03474
        // joint4 -> joint5 x: 0.10985->0.13224 z: 0.07503->0.027075
//        joint3_to_joint4_matrix = ToTransform(-0.0264, 0, 0.02235);
        joint4_to_joint5_matrix = ToTransform(0.13224, 0, 0.027075);
    }
    // [v3]
    joint3_to_joint4_matrix = ToTransform();
    joint4_to_joint5_matrix = ToTransform();

    Eigen::Matrix4d pose_matrix = world_to_joint_base_matrix * joint_base_to_joint0_matrix *
        joint0_to_joint1_matrix * joint1_to_joint2_matrix * joint2_to_joint3_matrix * joint3_to_joint4_matrix;
    Eigen::Matrix4d imu_diff_matrix(ToTransform(pose_matrix(0, 3), pose_matrix(1, 3), pose_matrix(2, 3),
                                                cur_imu_diff_quatd));
    imu_diff_matrix = pose_matrix.inverse() * imu_diff_matrix;
    pose_matrix = pose_matrix * imu_diff_matrix * joint4_to_joint5_matrix * gripper_length_matrix;

    PublishTF(world_to_joint_base_matrix, "world", node_name+"_joint_base");
    PublishTF(joint_base_to_joint0_matrix, node_name+"_joint_base", node_name+"_joint0");
    PublishTF(joint0_to_joint1_matrix, node_name+"_joint0", node_name+"_joint1");
    PublishTF(joint1_to_joint2_matrix, node_name+"_joint1", node_name+"_joint2");
    PublishTF(joint2_to_joint3_matrix, node_name+"_joint2", node_name+"_joint3");
    PublishTF(joint3_to_joint4_matrix, node_name+"_joint3", node_name+"_joint4");
    PublishTF(imu_diff_matrix, node_name+"_joint4", node_name+"_joint_end");
    PublishTF(joint4_to_joint5_matrix, node_name+"_joint_end", node_name+"_joint_trigger");
    if (gripper_length > 0)
    {
        PublishTF(gripper_length_matrix, node_name+"_joint_trigger", node_name+"_joint_gripper");
        PublishTF(gripper_left_matrix, node_name+"_joint_trigger", node_name+"_joint_gripper_left");
        PublishTF(gripper_right_matrix, node_name+"_joint_trigger", node_name+"_joint_gripper_right");
    }

    cur_pose_matrix = pose_matrix;
    PublishRemotePose();
}

void TegoRosMessenger::PublishRemotePose()
{
    static bool firstInit= true;
    if (update_base_pose)
    {
        firstInit = false;
        update_base_pose = false;
        remote_base_pose_matrix = cur_pose_matrix;
    }
    if (firstInit)
        return;

    double cur_zoom_factor = !use_piper_mode && use_pose_dt_zoom ? zoom_factor : 1.0;
    Eigen::Matrix4d pose_matrix(remote_base_pose_matrix);
    Eigen::Matrix4d dt_matrix = remote_base_pose_matrix.inverse() * cur_pose_matrix;
    dt_matrix.block<3, 1>(0, 3) *= cur_zoom_factor;
    pose_matrix *= dt_matrix;

    tf2::Matrix3x3 rotation_matrix(
        pose_matrix(0, 0), pose_matrix(0, 1), pose_matrix(0, 2),
        pose_matrix(1, 0), pose_matrix(1, 1), pose_matrix(1, 2),
        pose_matrix(2, 0), pose_matrix(2, 1), pose_matrix(2, 2)
    );
    tf2::Quaternion q;
    rotation_matrix.getRotation(q);
    PublishTF(pose_matrix, "world", node_name+"_gripper_pose");
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = pose_matrix(0, 3);
    pose_msg.pose.position.y = pose_matrix(1, 3);
    pose_msg.pose.position.z = pose_matrix(2, 3);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    tego_pose_publisher_->publish(pose_msg);
}

void TegoRosMessenger::AnalyzeKeyEventAction(uint8_t key_id, uint8_t click_type)
{
    constexpr uint8_t KEY_LEFT = 0x01;
    constexpr uint8_t KEY_RIGHT = 0x02;
    constexpr uint8_t CLICK_DOUBLE = 0x02;
    constexpr uint8_t CLICK_LONG = 0x03;

    bool is_left_handle = node_name.find("_l") != std::string::npos ||
                          node_name.find("left") != std::string::npos;
    bool is_right_handle = node_name.find("_r") != std::string::npos ||
                           node_name.find("right") != std::string::npos;
    bool is_inside_key = (!is_left_handle && !is_right_handle) ||
                         (is_left_handle && key_id == KEY_RIGHT) ||
                         (is_right_handle && key_id == KEY_LEFT);
    if (!is_inside_key)
    {
        return;
    }

    if (click_type == CLICK_DOUBLE)
    {
        if (teleop_enable.load())
        {
            RosDurationSleep(double_press_timeout);
        }
        makeThread([this]
        {
            const bool next_trigger_action = !is_trigger_action.load();
            RCLCPP_INFO(this->get_logger(), "[side key trigger] -> %s", Bool2String[next_trigger_action].c_str());
            this->set_parameter(rclcpp::Parameter("trigger_action", next_trigger_action));
        });
        return;
    }

    if (click_type == CLICK_LONG)
    {
        if (!teleop_enable.load())
        {
            RCLCPP_INFO(this->get_logger(), "[side key init]");
            makeThread([this]
            {
                RosDurationSleep(double_press_timeout);
                if (!teleop_enable.load())
                {
                    this->set_parameter(rclcpp::Parameter("init_action", true));
                }
                else
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[side key init] canceled because teleop is enabled");
                }
            });
        }
        else
        {
            std::cout << "pass" << std::endl;
        }
        return;
    }
}

bool TegoRosMessenger::CheckTimeout(rclcpp::Time time, double timeout)
{
    return fabs((this->now() - time).seconds()) >= timeout;
}

bool TegoRosMessenger::CheckTriggerStatus(bool cur_trigger_status)
{
    return cur_trigger_status == teleop_enable.load();
}

void TegoRosMessenger::SplitRosParams(const std::string& data, std::vector<double>& vec)
{
    vec.clear();
    std::vector<double> parsed_values;
    if (data.empty())
        throw std::invalid_argument("parameter must not be empty");

    std::stringstream ss(data);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        size_t start = token.find_first_not_of(" \t\r\n");
        size_t end = token.find_last_not_of(" \t\r\n");
        if (start == std::string::npos)
            throw std::invalid_argument("parameter contains an empty value");

        std::string trimmed = token.substr(start, end - start + 1);
        size_t consumed = 0;
        double value = 0.0;
        try
        {
            value = std::stod(trimmed, &consumed);
        }
        catch (const std::exception&)
        {
            throw std::invalid_argument("parameter contains a non-numeric value: " + trimmed);
        }
        if (consumed != trimmed.size())
            throw std::invalid_argument("parameter contains a non-numeric value: " + trimmed);
        if (!std::isfinite(value))
            throw std::invalid_argument("parameter must contain only finite values: " + trimmed);
        parsed_values.push_back(value);
    }

    if (parsed_values.empty() || data.back() == ',')
        throw std::invalid_argument("parameter must contain at least one value");
    vec.swap(parsed_values);
}

void TegoRosMessenger::SetHandleMotor(bool run)
{
    HandleMotorControlCmd cmd;
    // run = false;
    cmd.motor_mode = run ? HandleMotorControlCmd::MotorMode::START :
                           HandleMotorControlCmd::MotorMode::STOP;
    tego_->SetHandleMotorCommand(cmd);
}

void TegoRosMessenger::SetHandlelight(HandleLightControlCmd::ColorId color)
{
    HandleLightControlCmd cmd;
    cmd.color_id = color;
    tego_->SetHandleLightCommand(cmd);
}

void TegoRosMessenger::ResetTeleopEnableThread()
{
    double timeout = 1.0;
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok() && !stopping_threads_.load())
    {
        {
            std::lock_guard<std::mutex> state_lock(teleop_state_mutex);
            if (teleop_enable.load() && CheckTimeout(last_get_teleop_enable_time, timeout))
            {
                std::lock_guard<std::mutex> publish_lock(remote_joint_publish_mutex);
                // Re-check after acquiring the publish lock so a concurrent
                // teleop-enable callback cannot be overwritten by a stale timeout.
                if (teleop_enable.load() &&
                    CheckTimeout(last_get_teleop_enable_time, timeout))
                {
                    RCLCPP_INFO(this->get_logger(), "can't get teleop_enable status");
                    teleop_enable.store(false);
                }
            }
        }
        loop_rate.sleep();
    }
}

void TegoRosMessenger::ResetTriggerActionThread()
{
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok() && !stopping_threads_.load())
    {
        bool should_reset = false;
        bool current_trigger_action = false;
        {
            std::lock_guard<std::mutex> lock(teleop_state_mutex);
            current_trigger_action = is_trigger_action.load();
            should_reset = !is_reconfig_run.load() &&
                           CheckTimeout(last_call_teleop_time, 0.5) &&
                           current_trigger_action != teleop_enable.load();
        }
        if (should_reset)
        {
            const bool next_trigger_action = !current_trigger_action;
            RCLCPP_INFO(this->get_logger(), "[reset trigger] -> %s", Bool2String[next_trigger_action].c_str());
            this->set_parameter(rclcpp::Parameter("trigger_action", next_trigger_action));
        }
        loop_rate.sleep();
    }
}

void TegoRosMessenger::PublishStateToROS()
{
    auto state = tego_->GetTegoState();
    rclcpp::Time now = this->now();

    // sim joint
    if (publish_sim_joint_)
    {
        sensor_msgs::msg::JointState msg;
        msg.position.assign(7, 0.0);
        sim_joint_publisher_->publish(msg);
    }
    // tego sensor status
    if (!check_get_ || memcmp(state.mt_sensor_get, mt_sensor_send, sizeof(state.mt_sensor_get)))
    {
        memcpy(mt_sensor_send, state.mt_sensor_get, sizeof(state.mt_sensor_get));
        sensor_msgs::msg::JointState joint_msg;
        tego_msgs::msg::MtSensorStatus status_msg;
        for (int i = 0; i < 7; ++i)
        {
            std::string name = "joint" + std::to_string(i);
            joint_msg.name.push_back(name);
            joint_msg.position.push_back(state.mt_sensor_states[i].angle_rad);
            angle_list[i] = state.mt_sensor_states[i].angle_deg / 180.0 * M_PI;

            tego_msgs::msg::SensorData sensor_data;
            sensor_data.angle_deg = state.mt_sensor_states[i].angle_deg;
            sensor_data.angle_rad = state.mt_sensor_states[i].angle_rad;
            sensor_data.voltage_x = state.mt_sensor_states[i].voltage_x;
            sensor_data.voltage_y = state.mt_sensor_states[i].voltage_y;
            status_msg.name.push_back(name);
            status_msg.data.push_back(sensor_data);
            status_msg.receive_count.push_back(state.mt_sensor_get[i]);
        }
        if (update_joint)
        {
            update_joint = false;
            angle0_base = angle_list[1];
            angle1_base = angle_list[2];
            angle2_base = angle_list[3];
        }
        joint_publisher_->publish(joint_msg);
        sensor_status_publisher_->publish(status_msg);
    }
    // handle status
    if (!check_get_ || handle_status_send != state.handle_status_get ||
        handle_calibration_send != state.handle_calibration_get ||
        key1_send != state.key1_get || key2_send != state.key2_get)
    {
        handle_status_send = state.handle_status_get;
        handle_calibration_send = state.handle_calibration_get;
        key1_send = state.key1_get;
        key2_send = state.key2_get;
        tego_msgs::msg::HandleState handle_msg;
        handle_msg.joystick_x = state.handle_state.x;
        handle_msg.joystick_y = state.handle_state.y;
        handle_msg.trigger_press_percentage = state.handle_state.trigger;
        handle_msg.handle_receive_count = state.handle_status_get;
        handle_msg.key1_click_type = state.key1_click_type;
        handle_msg.key2_click_type = state.key2_click_type;
        handle_msg.key1_receive_count = state.key1_get;
        handle_msg.key2_receive_count = state.key2_get;
        handle_msg.calibration_receive_count = state.handle_calibration_get;
        handle_msg.calibration_x = state.handle_calibration_state.x;
        handle_msg.calibration_y = state.handle_calibration_state.y;
        handle_msg.calibration_trigger = state.handle_calibration_state.trigger;
        handle_msg.calibration_all = state.handle_calibration_state.all;
        trigger_press_percentage = state.handle_state.trigger;
        handle_state_publisher_->publish(handle_msg);
        static double last_gripper_length = -1;
        double cur_length = GetGripperWidth();
        if (is_nero && fabs(last_gripper_length - cur_length) > 0.003)
        {
            last_gripper_length = cur_length;
            sensor_msgs::msg::JointState gripper_msg;
            gripper_msg.header.stamp = now;
            gripper_msg.name = {"gripper"};
            gripper_msg.position = {GetGripperWidth()};
            gripper_msg.effort.push_back(3.0);
            nero_gripper_publisher_->publish(gripper_msg);
        }
    }
    // side key event
    if (key_event_send != state.key_event_get)
    {
        key_event_send = state.key_event_get;
        AnalyzeKeyEventAction(state.key_event_state.key_id, state.key_event_state.click_type);
    }
    // imu
    bool imu_init = state.imu_linear_acceleration_get != 0 ||
                    state.imu_angular_velocity_get != 0 ||
                    state.imu_orientation_get != 0;
    bool imu_get = imu_linear_acceleration_send != state.imu_linear_acceleration_get ||
                   imu_angular_velocity_send != state.imu_angular_velocity_get ||
                   imu_orientation_send != state.imu_orientation_get;
    if (!check_get_ || (imu_init && imu_get))
    {
        imu_linear_acceleration_send = state.imu_linear_acceleration_get;
        imu_angular_velocity_send = state.imu_angular_velocity_get;
        imu_orientation_send = state.imu_orientation_get;

        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = now;
        imu_msg->header.frame_id = "base_link";
        imu_msg->linear_acceleration.x = state.imu_linear_acceleration_state.x;
        imu_msg->linear_acceleration.y = state.imu_linear_acceleration_state.y;
        imu_msg->linear_acceleration.z = state.imu_linear_acceleration_state.z;
        imu_msg->angular_velocity.x = state.imu_angular_velocity_state.x;
        imu_msg->angular_velocity.y = state.imu_angular_velocity_state.y;
        imu_msg->angular_velocity.z = state.imu_angular_velocity_state.z;
        imu_msg->orientation.w = state.imu_orientation_state.w;
        imu_msg->orientation.x = state.imu_orientation_state.x;
        imu_msg->orientation.y = state.imu_orientation_state.y;
        imu_msg->orientation.z = state.imu_orientation_state.z;

        imu_publisher_->publish(*imu_msg);
        ImuCallback(imu_msg);
    }

    if (!update_joint)
        UpdateAndPublishPose();
}

} // namespace tegocore
