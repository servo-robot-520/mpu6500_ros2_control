#include "mpu6500_ros2_control/mpu6500_iio_device.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

namespace mpu6500_ros2_control {
    namespace {

        /// @brief IIO 角速度原始值属性名，索引 0/1/2 分别对应传感器 x/y/z。
        constexpr std::array<const char*, 3> kGyroChannels{
            "in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw"};
        /// @brief IIO 加速度原始值属性名，索引 0/1/2 分别对应传感器 x/y/z。
        constexpr std::array<const char*, 3> kAccelChannels{
            "in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw"};
        constexpr std::array<char, 3> kAxisLetters{'x', 'y', 'z'};

        /// @brief 去掉首尾空白字符。
        std::string trim(const std::string& text) {
            const auto begin = text.find_first_not_of(" \t\r\n");
            if (begin == std::string::npos) {
                return {};
            }
            const auto end = text.find_last_not_of(" \t\r\n");
            return text.substr(begin, end - begin + 1);
        }

        /// @brief 读取一个文本属性文件。
        Result read_text_file(const std::string& path, std::string* text) {
            errno = 0;
            std::ifstream file(path);
            if (!file.is_open()) {
                const int error_number = errno;
                return Result::failure("无法打开 " + path +
                                       (error_number != 0 ? ": " + std::string(std::strerror(error_number)) : ""));
            }
            std::ostringstream buffer;
            buffer << file.rdbuf();
            if (file.bad()) {
                return Result::failure("读取 " + path + " 失败");
            }
            *text = buffer.str();
            return Result::success();
        }

        /// @brief 解析十进制浮点数，要求整段文本都是数字。
        Result parse_double(const std::string& text, double* value) {
            const std::string trimmed = trim(text);
            if (trimmed.empty()) {
                return Result::failure("属性值为空");
            }
            errno = 0;
            char* end = nullptr;
            const double parsed = std::strtod(trimmed.c_str(), &end);
            if (errno != 0 || end == trimmed.c_str() || *end != '\0' || !std::isfinite(parsed)) {
                return Result::failure("无法解析数值 '" + trimmed + "'");
            }
            *value = parsed;
            return Result::success();
        }

        /// @brief 解析十进制整数，要求整段文本都是数字。
        Result parse_integer(const std::string& text, long long* value) {
            const std::string trimmed = trim(text);
            if (trimmed.empty()) {
                return Result::failure("属性值为空");
            }
            errno = 0;
            char* end = nullptr;
            const long long parsed = std::strtoll(trimmed.c_str(), &end, 10);
            if (errno != 0 || end == trimmed.c_str() || *end != '\0') {
                return Result::failure("无法解析整数 '" + trimmed + "'");
            }
            *value = parsed;
            return Result::success();
        }

        /// @brief 把 IIO 设备路径解析为 sysfs 目录。
        ///
        /// /dev/iio:deviceN 字符设备本身不用于读取属性；同名的 sysfs 目录位于
        /// /sys/bus/iio/devices/ 下。也允许直接传入该目录或模拟目录。
        Result resolve_device_directory(const std::string& path, std::string* directory) {
            if (path.empty()) {
                return Result::failure("IIO 设备路径为空");
            }

            std::error_code error;
            const std::filesystem::path input(path);
            if (std::filesystem::is_directory(input, error)) {
                *directory = input.string();
                return Result::success();
            }

            const std::string basename = input.filename().string();
            if (basename.rfind("iio:device", 0) == 0) {
                const std::filesystem::path candidate =
                    std::filesystem::path("/sys/bus/iio/devices") / basename;
                if (std::filesystem::is_directory(candidate, error)) {
                    *directory = candidate.string();
                    return Result::success();
                }
            }

            return Result::failure("IIO 设备目录不存在: " + path +
                                   "（应指向 /dev/iio:deviceN 或 /sys/bus/iio/devices/iio:deviceN）");
        }

        /// @brief 打开一个只读属性文件。
        Result open_attribute(const std::string& path, int* fd) {
            errno = 0;
            const int opened = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
            if (opened < 0) {
                return Result::failure("无法打开 " + path + ": " + std::strerror(errno));
            }
            *fd = opened;
            return Result::success();
        }

        /// @brief 写入一个 sysfs 属性文件。
        ///
        /// 使用 O_WRONLY 而不是 ofstream：sysfs 属性文件不能通过打开操作创建，
        /// 缺失时应明确失败，而不是在普通目录里意外生成同名文件。
        Result write_text_file(const std::string& path, const std::string& text) {
            errno = 0;
            const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
            if (fd < 0) {
                return Result::failure("无法打开 " + path + ": " + std::strerror(errno));
            }

            std::size_t offset = 0;
            while (offset < text.size()) {
                const ssize_t written = ::write(fd, text.data() + offset, text.size() - offset);
                if (written < 0) {
                    if (errno == EINTR) {
                        continue;
                    }
                    const int error_number = errno;
                    ::close(fd);
                    return Result::failure("写入 " + path + " 失败: " +
                                           std::strerror(error_number));
                }
                offset += static_cast<std::size_t>(written);
            }

            if (::close(fd) != 0) {
                return Result::failure("关闭 " + path + " 失败: " + std::strerror(errno));
            }
            return Result::success();
        }

        /// @brief 读取某一轴的 scale：优先使用逐通道属性，其次使用共享属性。
        Result read_scale(const std::string& directory, const char* prefix, const int axis, double* scale) {
            const std::string per_channel =
                directory + "/in_" + prefix + "_" + std::string(1, kAxisLetters[axis]) + "_scale";
            const std::string shared = directory + "/in_" + prefix + "_scale";

            std::string text;
            auto result = read_text_file(per_channel, &text);
            if (!result) {
                result = read_text_file(shared, &text);
            }
            if (!result) {
                return Result::failure("缺少 IIO scale 属性 " + per_channel + " / " + shared +
                                       "；也可以在插件参数中显式设置 gyro_scale / accel_scale");
            }

            double value = 0.0;
            if (const auto parsed = parse_double(text, &value); !parsed) {
                return Result::failure("解析 " + per_channel + " 失败: " + parsed.error());
            }
            if (!(value > 0.0)) {
                return Result::failure("IIO scale 必须为正数: " + per_channel);
            }
            *scale = value;
            return Result::success();
        }

        /// @brief 从已打开的属性文件读取一个整数，offset 固定为 0。
        Result read_raw_value(const int fd, const std::string& path, long long* value) {
            std::array<char, 64> buffer{};
            ssize_t count = 0;
            do {
                count = ::pread(fd, buffer.data(), buffer.size() - 1, 0);
            } while (count < 0 && errno == EINTR);

            if (count < 0) {
                return Result::failure("读取 " + path + " 失败: " + std::strerror(errno));
            }
            if (count == 0) {
                return Result::failure("读取 " + path + " 返回空内容");
            }
            buffer[static_cast<std::size_t>(count)] = '\0';
            return parse_integer(buffer.data(), value);
        }

    } // namespace

    Mpu6500IioDevice::~Mpu6500IioDevice() {
        close();
    }

    void Mpu6500IioDevice::close() {
        for (auto& fd : fds_) {
            if (fd >= 0) {
                ::close(fd);
                fd = -1;
            }
        }
        device_directory_.clear();
        device_name_.clear();
    }

    bool Mpu6500IioDevice::is_open() const {
        return !device_directory_.empty() && fds_[0] >= 0;
    }

    Result Mpu6500IioDevice::open(const IioDeviceOptions& options, IioDeviceInfo* info) {
        close();
        if (info != nullptr) {
            *info = IioDeviceInfo{};
        }

        if (options.device_path.empty()) {
            return Result::failure("iio_device 未配置");
        }

        // 轴映射必须是 x/y/z 的一个排列，符号只能是 +1/-1。
        std::array<bool, 3> used{};
        for (int axis = 0; axis < 3; ++axis) {
            const int source = options.axis_map[axis];
            if (source < 0 || source > 2 || used[source]) {
                return Result::failure("axis_map 必须是 x/y/z 的一个排列");
            }
            used[source] = true;
            if (options.axis_sign[axis] != 1.0 && options.axis_sign[axis] != -1.0) {
                return Result::failure("axis_sign 只能取 +1 或 -1");
            }
        }

        auto result = resolve_device_directory(options.device_path, &device_directory_);
        if (!result) {
            return result;
        }

        IioDeviceInfo opened_info;
        opened_info.device_directory = device_directory_;

        std::string name_text;
        if (result = read_text_file(device_directory_ + "/name", &name_text); !result) {
            close();
            return result;
        }
        device_name_ = trim(name_text);
        opened_info.device_name = device_name_;

        const bool name_matches =
            options.expected_name.empty()
                ? (device_name_.find("mpu6500") != std::string::npos ||
                   device_name_.find("mpu6050") != std::string::npos)
                : device_name_ == options.expected_name;
        if (!name_matches) {
            if (options.verify_name) {
                close();
                return Result::failure("IIO 设备名不匹配: 实际 '" + device_name_ + "', 期望 '" +
                                       (options.expected_name.empty() ? "mpu6500/mpu6050"
                                                                       : options.expected_name) +
                                       "'；如确认设备正确可设置 verify_name=false");
            }
            opened_info.warnings.push_back("IIO 设备名 '" + device_name_ + "' 不在已知 MPU6500 名称中");
        }

        // 一次性打开 6 个 raw 属性，read() 中只做 pread()。
        for (int axis = 0; axis < 3; ++axis) {
            const std::string gyro_path = device_directory_ + "/" + kGyroChannels[axis];
            if (result = open_attribute(gyro_path, &fds_[axis]); !result) {
                close();
                return result;
            }
            const std::string accel_path = device_directory_ + "/" + kAccelChannels[axis];
            if (result = open_attribute(accel_path, &fds_[3 + axis]); !result) {
                close();
                return result;
            }
        }

        for (int axis = 0; axis < 3; ++axis) {
            if (options.gyro_scale > 0.0) {
                gyro_scale_[axis] = options.gyro_scale;
            } else if (result = read_scale(device_directory_, "anglvel", axis, &gyro_scale_[axis]); !result) {
                close();
                return result;
            }

            if (options.accel_scale > 0.0) {
                accel_scale_[axis] = options.accel_scale;
            } else if (result = read_scale(device_directory_, "accel", axis, &accel_scale_[axis]); !result) {
                close();
                return result;
            }
        }

        if (options.sampling_frequency_hz > 0.0) {
            std::ostringstream text;
            text << std::setprecision(9) << options.sampling_frequency_hz << '\n';
            if (result = write_text_file(device_directory_ + "/sampling_frequency", text.str()); !result) {
                if (options.strict_sampling_frequency) {
                    close();
                    return Result::failure("写入 sampling_frequency 失败: " + result.error());
                }
                opened_info.warnings.push_back("未能写入 sampling_frequency: " + result.error());
            }
        }

        axis_map_ = options.axis_map;
        axis_sign_ = options.axis_sign;
        opened_info.gyro_scale = gyro_scale_;
        opened_info.accel_scale = accel_scale_;

        if (info != nullptr) {
            *info = std::move(opened_info);
        }
        return Result::success();
    }

    Result Mpu6500IioDevice::read(ImuSample* sample) {
        if (!is_open()) {
            return Result::failure("IIO 设备未打开");
        }
        if (sample == nullptr) {
            return Result::failure("read() 的输出指针为空");
        }

        std::array<long long, kChannelCount> raw{};
        for (int index = 0; index < kChannelCount; ++index) {
            if (const int fd = fds_[index]; fd < 0) {
                return Result::failure("IIO 通道未打开");
            }
            const char* channel = index < 3 ? kGyroChannels[index] : kAccelChannels[index - 3];
            const std::string path = device_directory_ + "/" + channel;
            if (const auto result = read_raw_value(fds_[index], path, &raw[index]); !result) {
                return result;
            }
        }

        std::array<double, 3> gyro_sensor{};
        std::array<double, 3> accel_sensor{};
        for (int axis = 0; axis < 3; ++axis) {
            gyro_sensor[axis] = static_cast<double>(raw[axis]) * gyro_scale_[axis];
            accel_sensor[axis] = static_cast<double>(raw[3 + axis]) * accel_scale_[axis];
        }

        ImuSample result{};
        for (int body_axis = 0; body_axis < 3; ++body_axis) {
            const int source_axis = axis_map_[body_axis];
            result.angular_velocity[body_axis] =
                axis_sign_[body_axis] * gyro_sensor[source_axis];
            result.linear_acceleration[body_axis] =
                axis_sign_[body_axis] * accel_sensor[source_axis];
        }

        for (const double value : result.angular_velocity) {
            if (!std::isfinite(value)) {
                return Result::failure("角速度包含非有限值");
            }
        }
        for (const double value : result.linear_acceleration) {
            if (!std::isfinite(value)) {
                return Result::failure("加速度包含非有限值");
            }
        }

        *sample = result;
        return Result::success();
    }

} // namespace mpu6500_ros2_control
