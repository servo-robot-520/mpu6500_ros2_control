#include "mpu6500_ros2_control/mpu6500_sensor_interface.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common_cpp/logging/rclcpp_log_handler.hpp"
#include "common_cpp/logging/log_interface/log_macro.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mpu6500_ros2_control {
    namespace {

        /// @brief 标准重力加速度，m/s²；用于判断加速度是否主要反映重力。
        constexpr double kGravity = 9.80665;

        /// @brief 判断加速度模长是否在 1 g 附近，可作为重力方向使用。
        ///
        /// 该判定与 OrientationFilter 内部的加速度校正条件一致；启动阶段必须
        /// 通过它才允许用加速度初始化姿态，避免机器人被抬起/加速时把运动
        /// 加速度误认为重力。
        bool acceleration_matches_gravity(const std::array<double, 3>& acceleration,
                                          const double tolerance) {
            const double norm = std::sqrt(acceleration[0] * acceleration[0] +
                acceleration[1] * acceleration[1] +
                acceleration[2] * acceleration[2]);
            const double relative_tolerance = std::max(0.0, tolerance);
            return std::isfinite(norm) && norm > 1e-6 &&
                std::abs(norm - kGravity) <= kGravity * relative_tolerance;
        }

        /// @brief 去掉首尾空白字符。
        std::string trim(const std::string& text) {
            const auto begin = text.find_first_not_of(" \t\r\n");
            if (begin == std::string::npos) {
                return {};
            }
            const auto end = text.find_last_not_of(" \t\r\n");
            return text.substr(begin, end - begin + 1);
        }

        /// @brief 解析整数并校验取值范围。
        Result parse_integer_parameter(const std::string& label, const std::string& text, const int min,
                                       const int max, int* out) {
            try {
                size_t consumed = 0;
                const long value = std::stol(text, &consumed);
                if (consumed != text.size()) {
                    return Result::failure(label + ": expected an integer, got '" + text + "'");
                }
                if (value < min || value > max) {
                    return Result::failure(label + ": " + text + " is outside [" + std::to_string(min) + ", " +
                        std::to_string(max) + "]");
                }
                *out = static_cast<int>(value);
                return Result::success();
            }
            catch (const std::exception&) {
                return Result::failure(label + ": expected an integer, got '" + text + "'");
            }
        }

        /// @brief 解析浮点数并校验取值范围。
        Result parse_real_parameter(const std::string& label, const std::string& text, const double min,
                                    const double max, double* out) {
            try {
                size_t consumed = 0;
                const double value = std::stod(text, &consumed);
                if (consumed != text.size()) {
                    return Result::failure(label + ": expected a number, got '" + text + "'");
                }
                if (!std::isfinite(value) || value < min || value > max) {
                    return Result::failure(label + ": " + text + " is outside [" + std::to_string(min) + ", " +
                        std::to_string(max) + "]");
                }
                *out = value;
                return Result::success();
            }
            catch (const std::exception&) {
                return Result::failure(label + ": expected a number, got '" + text + "'");
            }
        }

        /// @brief 解析布尔值（true/false、True/False、1/0）。
        Result parse_boolean_parameter(const std::string& label, const std::string& text, bool* out) {
            if (text == "true" || text == "True" || text == "1") {
                *out = true;
                return Result::success();
            }
            if (text == "false" || text == "False" || text == "0") {
                *out = false;
                return Result::success();
            }
            return Result::failure(label + ": expected 'true' or 'false', got '" + text + "'");
        }

        /// @brief 解析形如 "x,y,z" 或 "x,-y,z" 的车体轴映射。
        Result parse_axis_map(const std::string& text, std::array<int, 3>* axis_map,
                              std::array<double, 3>* axis_sign) {
            std::vector<std::string> tokens;
            size_t start = 0;
            while (true) {
                const size_t comma = text.find(',', start);
                tokens.push_back(trim(text.substr(start, comma == std::string::npos
                                                  ? std::string::npos
                                                  : comma - start)));
                if (comma == std::string::npos) {
                    break;
                }
                start = comma + 1;
            }
            if (tokens.size() != 3) {
                return Result::failure("axis_map must contain three axes separated by commas, for example 'x,y,z'");
            }

            std::array<bool, 3> used{};
            for (int axis = 0; axis < 3; ++axis) {
                const std::string& token = tokens[static_cast<std::size_t>(axis)];
                size_t position = 0;
                double sign = 1.0;
                if (position < token.size() && (token[position] == '+' || token[position] == '-')) {
                    sign = token[position] == '-' ? -1.0 : 1.0;
                    ++position;
                }
                if (position + 1 != token.size()) {
                    return Result::failure("axis_map item " + std::to_string(axis + 1) +
                        " is invalid: '" + token + "'");
                }
                int source_axis = -1;
                switch (token[position]) {
                case 'x':
                    source_axis = 0;
                    break;
                case 'y':
                    source_axis = 1;
                    break;
                case 'z':
                    source_axis = 2;
                    break;
                default:
                    return Result::failure("axis_map item " + std::to_string(axis + 1) +
                        " must be x/y/z: '" + token + "'");
                }
                if (used[static_cast<std::size_t>(source_axis)]) {
                    return Result::failure("The axis_map cannot reuse the same sensor axis.");
                }
                used[static_cast<std::size_t>(source_axis)] = true;
                (*axis_map)[static_cast<std::size_t>(axis)] = source_axis;
                (*axis_sign)[static_cast<std::size_t>(axis)] = sign;
            }
            return Result::success();
        }

        /// @brief 支持的状态接口名称集合。
        const std::set<std::string>& supported_state_interfaces() {
            static const std::set<std::string> kSupported{
                "orientation.x",
                "orientation.y",
                "orientation.z",
                "orientation.w",
                "angular_velocity.x",
                "angular_velocity.y",
                "angular_velocity.z",
                "linear_acceleration.x",
                "linear_acceleration.y",
                "linear_acceleration.z",
                "orientation_valid",
            };
            return kSupported;
        }

    } // namespace

    hardware_interface::CallbackReturn MPU6500SensorInterface::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // common_cpp 日志后端是进程级的；同一进程中的多个 IMU 组件必须使用相同的
        // log.* 配置（默认配置即可），否则第二次初始化会报错。
        auto logging_parameters = info_.hardware_parameters;
        logging_parameters.try_emplace("log.logger_name", "mpu6500_ros2_control");
        logging_parameters.try_emplace("log.node_name", "mpu6500_ros2_control");
        try {
            logging::ros::RclcppLogHandler::init_from_map(logging_parameters);
        }
        catch (const std::exception& error) {
            RCLCPP_ERROR(rclcpp::get_logger("mpu6500_ros2_control"),
                         "Failed to initialize common_cpp logging: %s", error.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const char* caller = __func__;
        auto fail = [caller](const std::string& message) {
            LOG_ERROR_WITH_FUNC(caller, "{}", message);
            return hardware_interface::CallbackReturn::ERROR;
        };

        device_.close();
        reset_runtime_state();

        if (info_.sensors.empty()) {
            return fail("A <sensor> must be declared within the <ros2_control type=\"sensor\"> of the URDF.");
        }
        if (info_.sensors.size() > 1) {
            return fail("One MPU6500SensorInterface instance can only correspond to one <sensor>; "
                "for dual IMUs, declare two independent <ros2_control type=\"sensor\"> components.");
        }

        const auto& sensor = info_.sensors.front();
        if (sensor.state_interfaces.empty()) {
            return fail("The sensor '" + sensor.name + "' has no declared state interfaces.");
        }
        for (const auto& state_interface : sensor.state_interfaces) {
            if (!supported_state_interfaces().contains(state_interface.name)) {
                return fail("sensor '" + sensor.name + "': unsupported state interface '" + state_interface.name +
                    "' (supported: orientation.x/y/z/w, angular_velocity.x/y/z, "
                    "linear_acceleration.x/y/z, orientation_valid)");
            }
        }

        if (const auto result = parse_options(caller);
            result != hardware_interface::CallbackReturn::SUCCESS) {
            return result;
        }

        LOG_INFO("configured sensor='{}': iio_device={} expected_name='{}' verify_name={} "
                 "max_read_failures={} valid_samples={} accel_gain={} integral_gain={}",
                 sensor.name, options_.device.device_path, options_.device.expected_name,
                 options_.device.verify_name, options_.max_read_failures, options_.valid_samples,
                 options_.filter.accel_gain, options_.filter.integral_gain);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MPU6500SensorInterface::parse_options(const char* caller) {
        auto fail = [caller](const std::string& message) {
            LOG_ERROR_WITH_FUNC(caller, "{}", message);
            return hardware_interface::CallbackReturn::ERROR;
        };

        options_ = Options{};
        const auto& parameters = info_.hardware_parameters;

        const auto device = parameters.find("iio_device");
        if (device == parameters.end() || device->second.empty()) {
            return fail("The required hardware parameter iio_device is missing (example: /dev/iio:device0)");
        }
        options_.device.device_path = device->second;

        if (const auto it = parameters.find("expected_name"); it != parameters.end()) {
            options_.device.expected_name = it->second;
        }

        if (const auto it = parameters.find("verify_name"); it != parameters.end()) {
            if (const auto result = parse_boolean_parameter("verify_name", it->second,
                                                            &options_.device.verify_name);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("axis_map"); it != parameters.end()) {
            if (const auto result = parse_axis_map(it->second, &options_.device.axis_map,
                                                   &options_.device.axis_sign);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("gyro_scale"); it != parameters.end()) {
            if (const auto result = parse_real_parameter("gyro_scale", it->second, 0.0, 1.0e6,
                                                         &options_.device.gyro_scale);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("accel_scale"); it != parameters.end()) {
            if (const auto result = parse_real_parameter("accel_scale", it->second, 0.0, 1.0e6,
                                                         &options_.device.accel_scale);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("sampling_frequency_hz"); it != parameters.end()) {
            if (const auto result = parse_real_parameter("sampling_frequency_hz", it->second, 0.0, 10000.0,
                                                         &options_.device.sampling_frequency_hz);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("strict_sampling_frequency"); it != parameters.end()) {
            if (const auto result = parse_boolean_parameter("strict_sampling_frequency", it->second,
                                                            &options_.device.strict_sampling_frequency);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("max_read_failures"); it != parameters.end()) {
            if (const auto result = parse_integer_parameter("max_read_failures", it->second, 0, 1000,
                                                            &options_.max_read_failures);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("orientation.accel_gain"); it != parameters.end()) {
            if (const auto result = parse_real_parameter("orientation.accel_gain", it->second, 0.0, 100.0,
                                                         &options_.filter.accel_gain);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("orientation.integral_gain"); it != parameters.end()) {
            if (const auto result = parse_real_parameter("orientation.integral_gain", it->second, 0.0, 100.0,
                                                         &options_.filter.integral_gain);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("orientation.accel_tolerance"); it != parameters.end()) {
            if (const auto result = parse_real_parameter("orientation.accel_tolerance", it->second, 0.0, 10.0,
                                                         &options_.filter.accel_tolerance);
                !result) {
                return fail(result.error());
            }
        }

        if (const auto it = parameters.find("orientation.valid_samples"); it != parameters.end()) {
            if (const auto result = parse_integer_parameter("orientation.valid_samples", it->second, 1, 10000,
                                                            &options_.valid_samples);
                !result) {
                return fail(result.error());
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void MPU6500SensorInterface::reset_runtime_state() {
        state_ = State{};
        filter_ = OrientationFilter{};
        consecutive_failures_ = 0;
        consecutive_valid_samples_ = 0;
        has_sample_ = false;
        last_read_time_ns_ = 0;
    }

    hardware_interface::CallbackReturn MPU6500SensorInterface::on_configure(
        const rclcpp_lifecycle::State& /* previous_state */) {
        const char* caller = __func__;
        auto fail = [caller](const std::string& message) {
            LOG_ERROR_WITH_FUNC(caller, "{}", message);
            return hardware_interface::CallbackReturn::ERROR;
        };

        device_.close();
        reset_runtime_state();

        IioDeviceInfo opened;
        if (const auto result = device_.open(options_.device, &opened); !result) {
            return fail(result.error());
        }
        for (const auto& warning : opened.warnings) {
            LOG_WARN_WITH_FUNC(caller, "{}", warning);
        }

        filter_ = OrientationFilter{options_.filter};
        LOG_INFO("IIO device ready: dir={} name={} gyro_scale=[{}, {}, {}] accel_scale=[{}, {}, {}]",
                 opened.device_directory, opened.device_name, opened.gyro_scale[0], opened.gyro_scale[1],
                 opened.gyro_scale[2], opened.accel_scale[0], opened.accel_scale[1],
                 opened.accel_scale[2]);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MPU6500SensorInterface::on_cleanup(const rclcpp_lifecycle::State& /* previous_state */) {
        device_.close();
        reset_runtime_state();
        LOG_INFO("IIO device closed");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MPU6500SensorInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
        return on_cleanup(previous_state);
    }

    hardware_interface::CallbackReturn MPU6500SensorInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MPU6500SensorInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    double* MPU6500SensorInterface::interface_pointer(const std::string& name) {
        if (name == "orientation.x") {
            return &state_.orientation[0];
        }
        if (name == "orientation.y") {
            return &state_.orientation[1];
        }
        if (name == "orientation.z") {
            return &state_.orientation[2];
        }
        if (name == "orientation.w") {
            return &state_.orientation[3];
        }
        if (name == "angular_velocity.x") {
            return &state_.angular_velocity[0];
        }
        if (name == "angular_velocity.y") {
            return &state_.angular_velocity[1];
        }
        if (name == "angular_velocity.z") {
            return &state_.angular_velocity[2];
        }
        if (name == "linear_acceleration.x") {
            return &state_.linear_acceleration[0];
        }
        if (name == "linear_acceleration.y") {
            return &state_.linear_acceleration[1];
        }
        if (name == "linear_acceleration.z") {
            return &state_.linear_acceleration[2];
        }
        if (name == "orientation_valid") {
            return &state_.orientation_valid;
        }
        return nullptr;
    }

    std::vector<hardware_interface::StateInterface> MPU6500SensorInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> interfaces;
        if (info_.sensors.size() != 1) {
            return interfaces;
        }
        const auto& sensor = info_.sensors.front();
        interfaces.reserve(sensor.state_interfaces.size());
        for (const auto& state_interface : sensor.state_interfaces) {
            double* value = interface_pointer(state_interface.name);
            if (value == nullptr) {
                continue;
            }
            interfaces.emplace_back(sensor.name, state_interface.name, value);
        }
        return interfaces;
    }

    double MPU6500SensorInterface::sample_period(const rclcpp::Time& time, const rclcpp::Duration& period) {
        if (const double period_seconds = period.seconds();
            std::isfinite(period_seconds) && period_seconds > 0.0 && period_seconds <= 1.0) {
            if (const int64_t now = time.nanoseconds(); now > 0) {
                last_read_time_ns_ = now;
            }
            return period_seconds;
        }

        const int64_t now = time.nanoseconds();
        if (now > 0 && last_read_time_ns_ > 0 && now > last_read_time_ns_) {
            const double measured = static_cast<double>(now - last_read_time_ns_) * 1.0e-9;
            last_read_time_ns_ = now;
            if (std::isfinite(measured) && measured > 0.0 && measured <= 1.0) {
                return measured;
            }
        }
        if (now > 0) {
            last_read_time_ns_ = now;
        }
        // 没有有效时间信息时按 50 Hz 处理；滤波器的 dt 只影响积分步长。
        return 0.02;
    }

    hardware_interface::return_type MPU6500SensorInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
        if (!device_.is_open()) {
            LOG_ERROR_WITH_FUNC(__func__, "read() is called when the IIO device is not open.");
            return hardware_interface::return_type::ERROR;
        }

        ImuSample sample;
        if (const auto result = device_.read(&sample); !result) {
            ++consecutive_failures_;
            // 首帧失败时没有可用的旧样本，直接报错；之后允许短暂沿用上一帧。
            if (!has_sample_ || consecutive_failures_ > options_.max_read_failures) {
                // 持续失败后姿态不再可信；控制器应立即停止依赖姿态的动作。
                state_.orientation_valid = 0.0;
                consecutive_valid_samples_ = 0;
                LOG_ERROR_WITH_FUNC(__func__, "IMU read failed ({} times consecutively): {}", consecutive_failures_,
                                    result.error());
                return hardware_interface::return_type::ERROR;
            }
            // 短暂的读取失败不清除已经建立的 orientation_valid（通常只有几十 ms），
            // 但启动阶段不能用失败帧凑收敛计数。
            if (state_.orientation_valid < 0.5) {
                consecutive_valid_samples_ = 0;
            }
            LOG_WARN_WITH_FUNC(__func__, "IMU read failed, using the previous frame ({}/{}): {}", consecutive_failures_,
                               options_.max_read_failures, result.error());
            return hardware_interface::return_type::OK;
        }
        consecutive_failures_ = 0;

        const double dt = sample_period(time, period);
        const bool accel_is_gravity =
            acceleration_matches_gravity(sample.linear_acceleration, options_.filter.accel_tolerance);

        // 启动阶段：只有加速度模长接近 1 g 时才允许用它初始化姿态。机器人被
        // 抬起、放在斜面上或正在运动时，首帧加速度不是纯重力；此时保持未初始化，
        // 而不是把运动加速度写成单位四元数/错误重力。
        if (!filter_.initialized()) {
            if (accel_is_gravity) {
                filter_.update(sample.angular_velocity, sample.linear_acceleration, dt);
                if (consecutive_valid_samples_ < options_.valid_samples) {
                    ++consecutive_valid_samples_;
                }
            }
        }
        else {
            filter_.update(sample.angular_velocity, sample.linear_acceleration, dt);
            // 已经收敛后，线性加速度只跳过本帧的加速度校正，不清除有效性；
            // 否则机器人一开始走路就会把 orientation_valid 打成 0。
            if (state_.orientation_valid < 0.5) {
                if (accel_is_gravity) {
                    if (consecutive_valid_samples_ < options_.valid_samples) {
                        ++consecutive_valid_samples_;
                    }
                }
                else {
                    consecutive_valid_samples_ = 0;
                }
            }
        }

        state_.angular_velocity = sample.angular_velocity;
        state_.linear_acceleration = sample.linear_acceleration;
        state_.orientation = filter_.orientation();

        // orientation_valid 是“滤波器已收敛且传感器健康”，不是“本帧数据新鲜度”：
        // - 启动时累计到 valid_samples 个重力样本才置 1；
        // - 置 1 后，运动中的线性加速度不清零；
        // - 持续读失败、cleanup/reset 时清零。
        // 控制器应 claim 此接口，在 0 时保持 hold/home，不要启动策略或让跌倒检测投票。
        if (!filter_.initialized()) {
            state_.orientation_valid = 0.0;
        }
        else if (state_.orientation_valid < 0.5 &&
            consecutive_valid_samples_ >= options_.valid_samples) {
            state_.orientation_valid = 1.0;
            LOG_INFO_WITH_FUNC(__func__, "orientation_valid set after {} valid gravity samples",
                               consecutive_valid_samples_);
        }

        has_sample_ = true;
        return hardware_interface::return_type::OK;
    }

} // namespace mpu6500_ros2_control

PLUGINLIB_EXPORT_CLASS(mpu6500_ros2_control::MPU6500SensorInterface,
                       hardware_interface::SensorInterface)
