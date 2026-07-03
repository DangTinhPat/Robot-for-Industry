#include <cmath>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

/*
 * ackermann_transform_node — lớp đệm giữa Nav2 (cmd_vel) và
 * ackermann_steering_controller (reference).
 *
 * Điểm mấu chốt: MPPI phát lệnh (v, ω) nhiễu từng chu kỳ, nhưng phần cứng
 * thật vận hành trong không gian (v, GÓC LÁI δ) — servo lái chỉ quay được
 * max_steer_rate (1.0 rad/s theo ros2_control.xacro). Node này vì vậy:
 *
 *   cmd_vel (v, ω) ──EMA──> đích (v_t, δ_t)   với δ = atan(L·ω / v)
 *                    slew v  : max_linear_accel / max_linear_decel (tách riêng)
 *                    slew δ  : max_steer_rate (đúng tốc độ servo thật)
 *                    xuất    : v, ω = v·tan(δ)/L  → LUÔN khả thi Ackermann
 *                              và servo LUÔN bám kịp → hết giật/saturate.
 *
 * Mọi tham số double đều đổi được lúc chạy: ros2 param set /ackermann_transform ...
 */
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
        // Speed / smoothing limits
        declare_parameter("max_linear_speed",  0.8);
        declare_parameter("max_linear_accel",  1.0);
        declare_parameter("max_linear_decel",  1.5);   // phanh được phép gắt hơn tăng tốc
        declare_parameter("max_steer_rate",    1.0);   // rad/s — khớp steering_velocity servo
        declare_parameter("max_angular_accel", 2.0);   // DEPRECATED — thay bằng max_steer_rate
        declare_parameter("smoothing_time_constant", 0.05);  // s — EMA lọc nhiễu MPPI, 0 = tắt
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
        last_pub_   = last_stamp_;

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz_),
            [this]() { publish_cb(); });

        // Cho phép tune trực tiếp lúc chạy (ros2 param set) — không cần restart
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                return on_params(params);
            });

        RCLCPP_INFO(get_logger(),
            "[AckermannTransform] ready  %s → %s  "
            "wheelbase=%.3fm  max_steer=%.1f°  steer_rate=%.2frad/s  min_turn_r=%.3fm",
            in_topic_.c_str(), out_topic_.c_str(),
            L_, max_steer_ * 180.0 / M_PI, steer_rate_, L_ / std::tan(max_steer_));
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
        L_          = get_parameter("wheelbase").as_double();
        track_      = get_parameter("track_width").as_double();
        r_          = get_parameter("wheel_radius").as_double();
        max_steer_  = get_parameter("max_steer_angle").as_double();
        max_v_      = get_parameter("max_linear_speed").as_double();
        max_av_     = get_parameter("max_linear_accel").as_double();
        max_dv_     = get_parameter("max_linear_decel").as_double();
        steer_rate_ = get_parameter("max_steer_rate").as_double();
        ema_tau_    = get_parameter("smoothing_time_constant").as_double();
        creep_      = get_parameter("rotation_creep_speed").as_double();
        deadzone_   = get_parameter("linear_deadzone").as_double();
        timeout_    = get_parameter("cmd_timeout").as_double();
        rate_hz_    = get_parameter("publish_rate").as_double();
        in_topic_   = get_parameter("input_topic").as_string();
        out_topic_  = get_parameter("output_topic").as_string();
    }

    rcl_interfaces::msg::SetParametersResult
    on_params(const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
            const auto & n = p.get_name();
            if (n == "max_linear_speed")        max_v_      = p.as_double();
            else if (n == "max_linear_accel")   max_av_     = p.as_double();
            else if (n == "max_linear_decel")   max_dv_     = p.as_double();
            else if (n == "max_steer_rate")     steer_rate_ = p.as_double();
            else if (n == "smoothing_time_constant") ema_tau_ = p.as_double();
            else if (n == "rotation_creep_speed") creep_    = p.as_double();
            else if (n == "linear_deadzone")    deadzone_   = p.as_double();
            else if (n == "cmd_timeout")        timeout_    = p.as_double();
            else if (n == "max_steer_angle")    max_steer_  = p.as_double();
            else continue;  // geometry/topics: đổi lúc chạy không có ý nghĩa
            RCLCPP_INFO(get_logger(), "param %s = %.3f", n.c_str(), p.as_double());
        }
        return result;
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
            RCLCPP_WARN(get_logger(), "cmd_vel chứa NaN/Inf — bỏ qua.");
            return;
        }
        raw_v_      = msg->linear.x;
        raw_w_      = msg->angular.z;
        last_stamp_ = get_clock()->now();
    }

    // (v, ω) → đích trong không gian (v, δ).
    // Pure-rotation (v≈0, ω≠0) → creep mode: tiến/lùi chậm với góc lái gắt.
    std::pair<double, double> to_v_delta(double v, double w) const
    {
        if (std::abs(v) < deadzone_) {
            if (std::abs(w) < 0.05 || std::abs(creep_) < 1e-9) return {0.0, 0.0};
            v = (raw_v_ < -deadzone_) ? -creep_ : creep_;
        }
        // ω = v·tan(δ)/L  ⇒  δ = atan(L·ω / v) — phép chia có dấu tự xử lý
        // đúng chiều lùi (v<0, cùng ω ⇒ δ ngược dấu)
        double delta = std::atan(L_ * w / v);
        return {v, std::clamp(delta, -max_steer_, max_steer_)};
    }

    static double slew(double current, double target, double max_step)
    {
        return current + std::clamp(target - current, -max_step, max_step);
    }

    void publish_cb()
    {
        auto now = get_clock()->now();
        // dt theo đồng hồ node (sim time nếu use_sim_time) — wall timer chạy
        // theo giờ thật, nếu RTF<1 mà dùng dt cố định thì slew bị nhanh ảo
        double dt = std::clamp((now - last_pub_).seconds(), 0.0, 3.0 / rate_hz_);
        last_pub_ = now;
        if (dt <= 0.0) dt = 1.0 / rate_hz_;

        // 1. Safety timeout
        double age = (now - last_stamp_).seconds();
        double tv = (age > timeout_) ? 0.0 : raw_v_;
        double tw = (age > timeout_) ? 0.0 : raw_w_;

        // 2. EMA lọc nhiễu lệnh MPPI (tau=0 → tắt)
        if (ema_tau_ > 1e-6) {
            double alpha = dt / (ema_tau_ + dt);
            ema_v_ += alpha * (tv - ema_v_);
            ema_w_ += alpha * (tw - ema_w_);
        } else {
            ema_v_ = tv;
            ema_w_ = tw;
        }

        // 3. Clamp tốc độ + quy về không gian (v, δ)
        double v_in = std::clamp(ema_v_, -max_v_, max_v_);
        auto [v_t, delta_t] = to_v_delta(v_in, ema_w_);

        // 4. Slew v: phanh (về 0 / đổi chiều) được gắt hơn tăng tốc
        bool braking = std::abs(v_t) < std::abs(curr_v_) || v_t * curr_v_ < 0.0;
        curr_v_ = slew(curr_v_, v_t, (braking ? max_dv_ : max_av_) * dt);

        // 5. Slew δ đúng tốc độ servo thật → lệnh luôn bám kịp được
        //    Khi dừng hẳn: đưa bánh lái về giữa cho lần xuất phát sau
        if (std::abs(curr_v_) < deadzone_ && std::abs(v_t) < deadzone_) {
            delta_t = 0.0;
        }
        curr_delta_ = slew(curr_delta_, delta_t, steer_rate_ * dt);

        // 6. Tái tạo ω từ (v, δ) — theo cấu trúc luôn khả thi Ackermann
        double v_out = (std::abs(curr_v_) < 1e-4) ? 0.0 : curr_v_;
        double w_out = v_out * std::tan(curr_delta_) / L_;

        geometry_msgs::msg::TwistStamped out;
        out.header.stamp    = now;
        out.header.frame_id = "base_link";
        out.twist.linear.x  = v_out;
        out.twist.angular.z = w_out;
        pub_->publish(out);
    }

    // Parameters
    double L_{0.21}, track_{0.217}, r_{0.05}, max_steer_{0.52};
    double max_v_{0.8}, max_av_{1.0}, max_dv_{1.5}, steer_rate_{1.0};
    double ema_tau_{0.05};
    double creep_{0.08}, deadzone_{0.015};
    double timeout_{0.5}, rate_hz_{50.0};
    std::string in_topic_{"/cmd_vel"};
    std::string out_topic_{"/ackermann_steering_controller/reference"};

    // State
    double raw_v_{0.0}, raw_w_{0.0};       // lệnh thô mới nhất từ Nav2
    double ema_v_{0.0}, ema_w_{0.0};       // sau lọc EMA
    double curr_v_{0.0}, curr_delta_{0.0}; // trạng thái xuất (v, góc lái)
    rclcpp::Time last_stamp_, last_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
