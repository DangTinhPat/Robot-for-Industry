#include <cmath>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class AckermannTransformNode : public rclcpp::Node
{
public:
    AckermannTransformNode()
    : Node("ackermann_transform")
    {
        // Geometry
        declare_parameter("wheelbase",        0.21);
        declare_parameter("track_width",      0.217);
        declare_parameter("wheel_radius",     0.05);
        declare_parameter("max_steer_angle",  0.52);
        // Speed limits
        declare_parameter("max_linear_speed", 0.8);
        declare_parameter("max_linear_accel", 1.0);
        declare_parameter("max_angular_accel", 2.0);
        // Pure-rotation
        declare_parameter("rotation_creep_speed", 0.08);
        declare_parameter("linear_deadzone",       0.015);
        // Safety
        declare_parameter("cmd_timeout",   0.5);
        declare_parameter("publish_rate",  50.0);
        // Topics
        declare_parameter("input_topic",  std::string("/cmd_vel"));
        declare_parameter("output_topic", std::string("/ackermann_steering_controller/reference"));

        load_params();

        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            in_topic_, 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_cb(msg); });

        pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(out_topic_, 10);

        last_stamp_ = get_clock()->now();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz_),
            [this]() { publish_cb(); });

        RCLCPP_INFO(get_logger(),
            "[AckermannTransform] ready  %s → %s  "
            "wheelbase=%.3fm  max_steer=%.1f°  min_turn_r=%.3fm",
            in_topic_.c_str(), out_topic_.c_str(),
            L_, max_steer_ * 180.0 / M_PI, L_ / std::tan(max_steer_));
    }

    ~AckermannTransformNode()
    {
        geometry_msgs::msg::TwistStamped stop;
        stop.header.stamp    = get_clock()->now();
        stop.header.frame_id = "base_link";
        pub_->publish(stop);
    }

private:
    void load_params()
    {
        L_         = get_parameter("wheelbase").as_double();
        track_     = get_parameter("track_width").as_double();
        r_         = get_parameter("wheel_radius").as_double();
        max_steer_ = get_parameter("max_steer_angle").as_double();
        max_v_     = get_parameter("max_linear_speed").as_double();
        max_av_    = get_parameter("max_linear_accel").as_double();
        max_aw_    = get_parameter("max_angular_accel").as_double();
        creep_     = get_parameter("rotation_creep_speed").as_double();
        deadzone_  = get_parameter("linear_deadzone").as_double();
        timeout_   = get_parameter("cmd_timeout").as_double();
        rate_hz_   = get_parameter("publish_rate").as_double();
        in_topic_  = get_parameter("input_topic").as_string();
        out_topic_ = get_parameter("output_topic").as_string();
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
            RCLCPP_WARN(get_logger(), "cmd_vel chứa NaN/Inf — bỏ qua.");
            return;
        }
        target_v_   = msg->linear.x;
        target_w_   = msg->angular.z;
        last_stamp_ = get_clock()->now();
    }

    // Ràng buộc Ackermann:  omega_max(v) = |v| * tan(max_steer) / L
    // Pure-rotation (v≈0, w≠0) → creep mode: inject tốc độ nhỏ +creep
    std::pair<double, double> ackermann_constrain(double v, double w)
    {
        if (std::abs(v) < deadzone_) {
            if (std::abs(w) < 0.05) return {0.0, 0.0};
            if (std::abs(creep_) < 1e-9) return {0.0, 0.0};
            // giữ hướng theo target_v_ để khớp ý định planner (tiến/lùi)
            v = (target_v_ < -deadzone_) ? -creep_ : creep_;
        }
        double w_max = std::abs(v) * std::tan(max_steer_) / L_;
        return {v, std::clamp(w, -w_max, w_max)};
    }

    void accel_limit(double tv, double tw, double dt)
    {
        double dv = max_av_ * dt;
        double dw = max_aw_ * dt;
        curr_v_ += std::clamp(tv - curr_v_, -dv, dv);
        curr_w_ += std::clamp(tw - curr_w_, -dw, dw);
    }

    void publish_cb()
    {
        auto now = get_clock()->now();
        double dt = 1.0 / rate_hz_;

        // 1. Safety timeout
        double age = (now - last_stamp_).seconds();
        double tv = (age > timeout_) ? 0.0 : target_v_;
        double tw = (age > timeout_) ? 0.0 : target_w_;

        // 2. Clamp tốc độ tối đa
        tv = std::clamp(tv, -max_v_, max_v_);

        // 3. Ràng buộc Ackermann (+ pure-rotation creep)
        auto [v, w] = ackermann_constrain(tv, tw);

        // 4. Giới hạn gia tốc
        accel_limit(v, w, dt);

        // 5. Tái áp ràng buộc sau smoothing
        //    v và w decel với tốc độ khác nhau → có thể vi phạm ràng buộc
        if (std::abs(curr_v_) >= deadzone_) {
            double w_max = std::abs(curr_v_) * std::tan(max_steer_) / L_;
            curr_w_ = std::clamp(curr_w_, -w_max, w_max);
        } else {
            curr_w_ = 0.0;
        }

        // 6. Publish
        geometry_msgs::msg::TwistStamped out;
        out.header.stamp    = now;
        out.header.frame_id = "base_link";
        out.twist.linear.x  = curr_v_;
        out.twist.angular.z = curr_w_;
        pub_->publish(out);
    }

    // Parameters
    double L_{0.21}, track_{0.217}, r_{0.05}, max_steer_{0.52};
    double max_v_{0.8}, max_av_{1.0}, max_aw_{2.0};
    double creep_{0.08}, deadzone_{0.015};
    double timeout_{0.5}, rate_hz_{50.0};
    std::string in_topic_{"/cmd_vel"};
    std::string out_topic_{"/ackermann_steering_controller/reference"};

    // State
    double curr_v_{0.0}, curr_w_{0.0};
    double target_v_{0.0}, target_w_{0.0};
    rclcpp::Time last_stamp_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
