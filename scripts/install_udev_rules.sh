#!/usr/bin/env bash
# 安装 MPU6500 稳定 IIO 设备别名 udev 规则。
# 用法：sudo bash install_udev_rules.sh   （或安装后在 share 目录里执行）
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 支持两种位置：源码树 scripts/ 旁边有 ../udev，或已安装的 share/mpu6500_ros2_control/
candidates=(
    "${here}/../udev/99-mpu6500-iio.rules"
    "${here}/99-mpu6500-iio.rules"
)
if command -v ros2 >/dev/null 2>&1; then
    prefix="$(ros2 pkg prefix mpu6500_ros2_control 2>/dev/null || true)"
    if [[ -n "${prefix}" ]]; then
        candidates+=("${prefix}/share/mpu6500_ros2_control/udev/99-mpu6500-iio.rules")
    fi
fi

src=""
for c in "${candidates[@]}"; do
    if [[ -f "${c}" ]]; then src="${c}"; break; fi
done

if [[ -z "${src}" ]]; then
    echo "error: 找不到 99-mpu6500-iio.rules" >&2
    exit 1
fi

dest="/etc/udev/rules.d/99-mpu6500-iio.rules"
install -m 0644 "${src}" "${dest}"
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=iio
sleep 0.3

echo "已安装 ${dest}"
echo "检查别名：ls -l /dev/mpu6500-0 /dev/mpu6500-1"
ls -l /dev/mpu6500-0 /dev/mpu6500-1 2>/dev/null || true
