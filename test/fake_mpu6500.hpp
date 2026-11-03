#pragma once

#include <unistd.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace mpu6500_ros2_control::test {

    /// @brief 基于临时目录的模拟 IIO sysfs 设备。
    ///
    /// 目录中包含 name、scale 与 6 个 raw 属性文件，接口与 Linux IIO 一致。
    /// 测试可以随时改写 raw 文件；已打开的文件描述符会用 pread() 读到新值。
    class FakeMpu6500 {
    public:
        /// MPU6500 ±500 dps 量程的典型灵敏度，rad/s per LSB。
        static constexpr double kGyroScale = 0.0002664600;
        /// MPU6500 ±4 g 量程的典型灵敏度，m/s² per LSB。
        static constexpr double kAccelScale = 0.0011971;
        /// 1 g 对应的加速度原始值。
        static constexpr long kGravityRaw = 8192;

        explicit FakeMpu6500(const std::string& name = "mpu6500") {
            static std::atomic<int> sequence{0};
            directory_ = std::filesystem::temp_directory_path() /
                ("mpu6500_fake_" + std::to_string(::getpid()) + "_" +
                 std::to_string(sequence.fetch_add(1)));
            std::filesystem::create_directories(directory_);

            set_name(name);
            write_file("in_anglvel_scale", format(kGyroScale));
            write_file("in_accel_scale", format(kAccelScale));
            set_raw_gyro(0, 0, 0);
            set_raw_accel(0, 0, kGravityRaw);
        }

        ~FakeMpu6500() {
            std::error_code error;
            std::filesystem::remove_all(directory_, error);
        }

        FakeMpu6500(const FakeMpu6500&) = delete;
        FakeMpu6500& operator=(const FakeMpu6500&) = delete;

        [[nodiscard]] std::string path() const {
            return directory_.string();
        }

        void set_name(const std::string& name) {
            write_file("name", name);
        }

        void set_raw_gyro(const long x, const long y, const long z) {
            write_file("in_anglvel_x_raw", std::to_string(x));
            write_file("in_anglvel_y_raw", std::to_string(y));
            write_file("in_anglvel_z_raw", std::to_string(z));
        }

        void set_raw_accel(const long x, const long y, const long z) {
            write_file("in_accel_x_raw", std::to_string(x));
            write_file("in_accel_y_raw", std::to_string(y));
            write_file("in_accel_z_raw", std::to_string(z));
        }

        void set_gyro(const double x, const double y, const double z) {
            set_raw_gyro(std::lround(x / kGyroScale), std::lround(y / kGyroScale),
                         std::lround(z / kGyroScale));
        }

        void set_accel(const double x, const double y, const double z) {
            set_raw_accel(std::lround(x / kAccelScale), std::lround(y / kAccelScale),
                          std::lround(z / kAccelScale));
        }

        void write_file(const std::string& name, const std::string& content) const {
            std::ofstream file(directory_ / name, std::ios::out | std::ios::trunc);
            if (!file.is_open()) {
                throw std::runtime_error("无法创建模拟 IIO 属性: " + name);
            }
            file << content;
        }

        void remove_file(const std::string& name) const {
            std::error_code error;
            std::filesystem::remove(directory_ / name, error);
        }

        [[nodiscard]] std::string read_file(const std::string& name) const {
            std::ifstream file(directory_ / name);
            if (!file.is_open()) {
                return {};
            }
            std::ostringstream buffer;
            buffer << file.rdbuf();
            return buffer.str();
        }

    private:
        static std::string format(const double value) {
            std::ostringstream text;
            text.precision(12);
            text << value;
            return text.str();
        }

        std::filesystem::path directory_;
    };

} // namespace mpu6500_ros2_control::test
