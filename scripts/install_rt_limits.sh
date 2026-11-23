#!/usr/bin/env bash
# 为运行 controller_manager 的用户开启实时调度所需的内核资源限制。
# PREEMPT_RT 内核默认可用，但普通用户默认 RLIMIT_RTPRIO=0、memlock 有限，
# 不放开时 controller_manager 设 SCHED_FIFO 会失败（std::system_error: Resource
# temporarily unavailable）。本脚本写入 /etc/security/limits.d/ 一次性配置。
#
# 用法：sudo bash install_rt_limits.sh [用户名]
#   用户名缺省取 $SUDO_USER，再缺省为 cat。
set -euo pipefail

target_user="${1:-${SUDO_USER:-cat}}"
conf="/etc/security/limits.d/90-mpu6500-rt.conf"

cat >"${conf}" <<EOF
# 由 mpu6500_ros2_control/scripts/install_rt_limits.sh 生成
${target_user}   -   rtprio     99
${target_user}   -   memlock    unlimited
EOF

echo "已写入 ${conf}:"
cat "${conf}"
echo
echo "注意：limits 在登录时由 PAM 应用。请重新登录（或重新建立 SSH 会话）后生效。"
echo "验证： bash -lc 'ulimit -r; ulimit -l'   期望输出 99 与 unlimited"
