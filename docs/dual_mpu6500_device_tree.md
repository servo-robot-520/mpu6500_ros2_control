# 双 MPU6500 设备树配置、编译与生效检查

本文说明在 **EmbedFire LubanCat-Zero（RK3566）** 上，把两颗 MPU6500 挂到 **SPI3** 的完整步骤：设备树 overlay 配置 → 编译成 `.dtbo` → 启用 → 检查是否生效。适用于 Linux 6.1.x（`rk356x`）内核。

> 本文只管"内核侧设备树 + IIO"，不涉及 ROS。ROS 功能包的构建/运行见仓库根目录 `README.md`。

---

## 1. 硬件连接（本工程约定）

两颗 MPU6500 共用 SPI3（M1 引脚组），用片选区分：

| 逻辑名 | 片选 | SPI 设备 | INT 引脚 | 对应 IIO（示例） |
|---|---|---|---|---|
| `imu_trunk`（躯干） | SPI3 **CS0** | `spi3.0` | **GPIO1_B2** | `/dev/iio:deviceN` |
| `imu_head`（头部） | SPI3 **CS1** | `spi3.1` | **GPIO2_C4** | `/dev/iio:deviceM` |

SPI3 M1 引脚（RK3566）：

| 信号 | GPIO | 说明 |
|---|---|---|
| CLK | GPIO4_C2 | `spi3m1_pins` |
| MOSI | GPIO4_C3 | 主机输出 |
| MISO | GPIO4_C5 | 主机输入 |
| CS0 | GPIO4_C6 | 低有效，`spi3_cs0` |
| CS1 | GPIO1_B3 | 低有效，由 `cs-gpios` 指定 |

> ⚠️ `iio:deviceN` 的编号随内核探测顺序变化，**不要写死**。请用第 5 节的 udev 别名 `/dev/mpu6500-0`、`/dev/mpu6500-1`，或先 `cat /sys/bus/iio/devices/iio:device*/name` 确认真实编号。

---

## 2. 设备树 overlay 源码

overlay 源码见同目录 [`mpu6500-overlay-dual.dts`](./mpu6500-overlay-dual.dts)。要点：

- `target = <&spi3>`：把 SPI3 控制器打开（`status = "okay"`），并使用 M1 引脚组 `spi3m1_pins`（高速用 `spi3m1_pins_hs`）。
- `cs-gpios = <&gpio4 0x16 0x01>, <&gpio1 0x0b 0x01>`：CS0 = GPIO4_C6、CS1 = GPIO1_B3，均低有效。
- `mpu6500@0`：`reg = <0>`（CS0，imu_trunk），`interrupt-parent = <&gpio1>`，`interrupts = <0x0a 0x04>`（GPIO1_B2 / bank1 pin10，电平高）。
- `mpu6500@1`：`reg = <1>`（CS1，imu_head），`interrupt-parent = <&gpio2>`，`interrupts = <0x14 0x04>`（GPIO2_C4 / bank2 pin20，电平高）。
- `compatible = "invensense,mpu6500"`，`spi-max-frequency = <0xf4240>`（1 MHz）。

> 若你的板子用的是 SPI3 的 CS0/CS1，但 INT 接的引脚不同，只需改对应节点的 `interrupt-parent` / `interrupts`。

---

## 3. 编译成 dtbo

板端一般自带 `dtc`（device-tree-compiler）；若没有：`sudo apt install device-tree-compiler`。

```bash
# 在 overlay 源码所在目录
dtc -@ -I dts -O dtb -o rk3566-lubancat-0-mpu6500-overlay.dtbo mpu6500-overlay-dual.dts
```

`-@` 是必须的：它会为 `&gpio1`、`&gpio2`、`&spi3` 等标签引用生成 `__fixups__`，overlay 才能在上电时正确解析。

编译成功后可先做一次反编译自检：

```bash
dtc -I dtb -O dts rk3566-lubancat-0-mpu6500-overlay.dtbo | head -40
```

---

## 4. 部署并启用 overlay

```bash
# 1) 备份现有 overlay（若有）
sudo cp -a /boot/dtb/overlay/rk3566-lubancat-0-mpu6500-overlay.dtbo \
           /boot/dtb/overlay/rk3566-lubancat-0-mpu6500-overlay.dtbo.bak.$(date +%s)

# 2) 安装新 dtbo
sudo cp rk3566-lubancat-0-mpu6500-overlay.dtbo /boot/dtb/overlay/
sync

# 3) 确认 uEnv 已启用该 overlay（二选一，取决于你的板子用哪个 uEnv）
grep -n mpu6500 /boot/uEnv/uEnvLubanCatZW.txt /boot/uEnv/uEnv.txt
#   应包含：
#   dtoverlay=/dtb/overlay/rk3566-lubancat-0-mpu6500-overlay.dtbo
```

若缺少该行，手动加入（与其它 `dtoverlay=` 同级）：

```
dtoverlay=/dtb/overlay/rk3566-lubancat-0-mpu6500-overlay.dtbo
```

然后重启：

```bash
sudo reboot
```

> 回滚：把 `.bak.*` 备份文件复制回原名覆盖即可，再重启。

---

## 5. 检查是否生效

重启后按下面顺序检查（任一环节不符合就往下排查）：

### 5.1 设备树已加载

```bash
# 找到 SPI3 节点（RK3566 上通常是 spi@fe630000）
ls /proc/device-tree | grep spi
cat /proc/device-tree/spi@fe630000/status          # 期望 okay
ls /proc/device-tree/spi@fe630000/ | grep mpu6500  # 期望 mpu6500@0 mpu6500@1
```

### 5.2 SPI 设备与驱动绑定

```bash
ls /sys/bus/spi/devices            # 期望出现 spi3.0  spi3.1
dmesg | grep -iE "mpu|invensense"  # 期望 inv-mpu6000-spi spi3.0 / spi3.1 完成 probe
```

正常时**不应**出现 `invalid whoami 0xff expected 0x70`（见第 7 节）。

### 5.3 IIO 设备出现

```bash
for d in /sys/bus/iio/devices/iio:device*; do
  printf '%s -> %s\n' "$d" "$(cat "$d/name" 2>/dev/null)"
done
```

期望看到 **两个** `mpu6500`（另外 `fe720000.saradc` 是本板 ADC，不是 IMU）。例如：

```
/sys/bus/iio/devices/iio:device0 -> fe720000.saradc
/sys/bus/iio/devices/iio:device1 -> mpu6500
/sys/bus/iio/devices/iio:device2 -> mpu6500
```

### 5.4 能读到有效数据

```bash
for d in /sys/bus/iio/devices/iio:device1 /sys/bus/iio/devices/iio:device2; do
  echo "== $d =="
  echo "accel_scale=$(cat $d/in_accel_scale) gyro_scale=$(cat $d/in_anglvel_scale)"
  echo "acc x=$(cat $d/in_accel_x_raw) y=$(cat $d/in_accel_y_raw) z=$(cat $d/in_accel_z_raw)"
done
```

静止时加速度合矢量应≈1g（≈9.8 m/s²）；若读到全 `0` 或 `4095/0` 之类异常值，多半是 SPI 数据线问题。

### 5.5 数据就绪中断（INT）在计数

```bash
# 观察中断计数：连续读几次原始值，再看计数是否增长
grep inv_mpu /proc/interrupts
for i in 1 2 3 4 5; do cat /sys/bus/iio/devices/iio:device1/in_accel_x_raw >/dev/null; done
grep inv_mpu /proc/interrupts
```

期望有两条 `rockchip_gpio_irq ... inv_mpu`，且读数据时计数增长（GPIO1_B2 与 GPIO2_C4 两个 IRQ）。

### 5.6 稳定设备别名（推荐）

安装 udev 规则，把 `iio:deviceN` 固定映射为 `/dev/mpu6500-0`、`/dev/mpu6500-1`：

```bash
sudo bash $(ros2 pkg prefix mpu6500_ros2_control)/share/mpu6500_ros2_control/scripts/install_udev_rules.sh
ls -l /dev/mpu6500-0 /dev/mpu6500-1
#   /dev/mpu6500-0 -> iio:deviceN(CS0)
#   /dev/mpu6500-1 -> iio:deviceM(CS1)
```

这样 ROS 侧（launch 默认值）就不用关心 `iio:deviceN` 的具体编号。

#### 5.6.1 从旧版 `trunk/head` 别名升级

早期规则生成的是 `/dev/mpu6500-trunk`、`/dev/mpu6500-head`。新版统一为
`/dev/mpu6500-0`（CS0）、`/dev/mpu6500-1`（CS1）。如果你已经按旧方式装过 udev，
更新时**只需替换规则文件并重新加载，不用改动设备树 / overlay**：

```bash
# 1) 备份旧规则，并覆盖为新的 99-mpu6500-iio.rules（同名文件直接覆盖）
sudo cp -a /etc/udev/rules.d/99-mpu6500-iio.rules \
            /etc/udev/rules.d/99-mpu6500-iio.rules.bak.$(date +%s)
sudo cp 99-mpu6500-iio.rules /etc/udev/rules.d/99-mpu6500-iio.rules

# 2) 删除旧别名残留软链
sudo rm -f /dev/mpu6500-trunk /dev/mpu6500-head

# 3) 重新加载规则并重放 iio 设备的 add 事件
sudo udevadm control --reload-rules
sudo udevadm trigger --verbose --action=add --subsystem-match=iio

# 4) 确认没有其它文件名的 mpu6500 规则仍在生效
grep -rl mpu6500 /etc/udev/rules.d/ || true
```

或者直接用安装脚本（会把新规则装到 `/etc/udev/rules.d/99-mpu6500-iio.rules`
并自动 reload + trigger，最后打印别名）：

```bash
sudo bash $(ros2 pkg prefix mpu6500_ros2_control)/share/mpu6500_ros2_control/scripts/install_udev_rules.sh
```

验证（期望分别指向 `spi3.0` / `spi3.1`）：

```bash
ls -l /dev/mpu6500-0 /dev/mpu6500-1
readlink -f /dev/mpu6500-0 /dev/mpu6500-1
udevadm info -a -n /dev/mpu6500-0 | grep -m1 KERNELS
```

> 若 `ls` 看不到新别名（少数情况下 add 重放不会重建符号链接），`sudo reboot`
> 一次即可，重启后必定生效。
>
> 更新后记得把用旧别名或旧参数路径的地方同步改掉：`duck_bringup imu.launch.py`
> 默认值已是 `/dev/mpu6500-0` / `/dev/mpu6500-1`；`trunk_iio_device` /
> `head_iio_device` 参数名仍保留，只是默认路径变了。

---

## 6. 启用后的 ROS 侧（简述）

设备树生效后，ROS 侧用本功能包自带的示例即可验证：

```bash
ros2 launch mpu6500_ros2_control mpu6500_dual_example.launch.py
# 或接入整机：
ros2 launch duck_bringup imu.launch.py
```

会看到 `imu_sensor_broadcaster_imu0/imu`、`imu_sensor_broadcaster_imu1/imu` 两个话题。默认示例配置里的 `frame_id` 是 `imu0` / `imu1`（示例 URDF 里的 link 就叫 imu0/imu1）。若要在 microduck_rev 整机里把这些数据挂到 `imu_trunk` / `imu_head` 连杆，需在整机 URDF 中包含对应的 `<ros2_control>` 传感器块，并把这些广播器的 `sensor_name` 设为 `imu0`/`imu1`、`frame_id` 改为 `imu_trunk`/`imu_head`（详见 README「把 imu0 / imu1 绑定到本体连杆」）。本仓库已按此实现：`duck_bringup/urdf/duck_with_mpu6500.urdf.xacro`（整机装配）+ `duck_bringup/config/imu_controllers.yaml`（frame_id 映射），由 `duck_bringup/launch/imu.launch.py` 启动。

---

## 7. 故障排查

| 现象 | 可能原因 / 处理 |
|---|---|
| `dmesg` 报 `invalid whoami 0xff expected 0x70 (MPU6500)` | SPI **数据线不通**。最常见是 **MOSI/MISO 接反**；其次查 CS、供电、焊接/排线、`spi-max-frequency` 过高（先降到 1 MHz）。 |
| `whoami` 读到 `0x68` | 那颗是 **MPU6050**（兼容件），驱动同样接受；若需要严格区分可设置 `expected_name`。 |
| 只有一路 `mpu6500` 出现 | 检查 CS1(GPIO1_B3) 是否被别的功能占用、`cs-gpios` 顺序、对应 `mpu6500@1` 的 `status`。 |
| `/proc/interrupts` 里 `inv_mpu` 计数不动 | INT 引脚与设备树不一致（`interrupt-parent`/`interrupts`），或 INT 未接线；对照第 1 节引脚表核对。 |
| 启动日志偶发一条 `WARNING: ... __handle_irq_event_percpu ... iio_trigger_generic_data_rdy_poll` | 电平触发 INT 的偶发告警，**不影响数据采集**；出现一次后通常不再刷屏。 |
| overlay 未生效（`/proc/device-tree` 里没有 mpu6500 节点） | `dtc` 编译时漏了 `-@`；或 uEnv 没有启用该 `dtoverlay=`；或 dtbo 没放到 `/boot/dtb/overlay/`。 |

---

## 8. 相关文件

- overlay 源码：[`mpu6500-overlay-dual.dts`](./mpu6500-overlay-dual.dts)
- udev 规则：`../udev/99-mpu6500-iio.rules`，安装脚本 `../scripts/install_udev_rules.sh`
- ROS 示例 launch：`../launch/mpu6500_dual_example.launch.py`
- ROS 侧详细说明：本包 `README.md`
