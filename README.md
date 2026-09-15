# UR7e SDK

UR7e + 工具端 Robotiq 2F-85 的 Python SDK。支持单臂、双臂、键盘、ARX5 主臂和模型动作序列；不依赖 ROS2、LeRobot、UMI 或 Gello 的安装。

## 安装与模拟运行

使用 Conda 管理环境。`environment.yml` 默认使用 Python 3.9，并安装 SDK 和 UR 硬件依赖。ARX5 模式的 Python 版本必须与 `arx5_interface` 编译版本一致；使用其他版本的绑定时，相应调整环境中的 Python 版本。

克隆或下载仓库后，在仓库根目录执行：

```bash
conda env create -f environment.yml
conda activate ur7e_sdk
ur7e observe
ur7e keyboard
ur7e policy --duration 5
ur7e --config configs/bimanual_sim.yaml policy --duration 5
```

默认 `configs/sim.yaml` 不连接机器人。`policy` 默认运行保持当前位姿的 `HoldPolicy`，用于检查完整调用链。模拟后端直接跟随目标，不模拟动力学、碰撞或接触。所有 CLI 命令在仓库根目录运行；其他目录使用 `--config` 的绝对路径。

以后使用时运行 `conda activate ur7e_sdk`。修改环境定义后，在仓库根目录运行 `conda env update -f environment.yml`。

## 硬件设置：UR7e + 工具端 2F-85

以下菜单按 **PolyScope 5.x 英文界面**书写。先在示教器的 **Local Control** 模式完成设置，最后再切 Remote Control。配置文件统一是 `configs/ur7e.yaml`，第一台机器人对应 `arms[0]`。

### 1. 设置机器人 IP

1. 用网线连接电脑和 UR 控制箱。
2. 示教器右上角 `☰ > Settings > System > Network`，读取或设置机器人 IP。例如机器人设为 `192.168.1.10`，子网掩码 `255.255.255.0`；电脑对应有线网卡设为 `192.168.1.20/24`，两者不能同 IP。
3. 把机器人 IP 填入 `configs/ur7e.yaml` 的 `arms[0].ip`。工具端 2F-85 **没有需要单独配置的 IP 或 Web 页面**，SDK 访问同一机器人 IP 的 `63352` 端口。
4. 在电脑运行 `ping -c 3 192.168.1.10`，应收到回复。这里的地址是配置示例，使用你实际设置的 IP。

### 2. 安装并设置 Robotiq URCap

1. 从 [Robotiq 支持页](https://robotiq.com/support/2f85-2f140-adaptive-robot-gripper) 选择 **Universal Robots > 2F-85 / 2F-140 > Software > Gripper Software**，下载与当前 PolyScope 兼容的 URCap 包。解压，将 `Robotiq_Grippers-版本号.urcap` 放入 U 盘并插到示教器。
2. 打开 `☰ > Settings > System > URCaps`，点击 `+`，选中该 `.urcap` 文件，点击 **Restart**。重启后应能看到 Gripper 安装页面；已经安装兼容版本可跳过这一步。
3. 打开 **`Installation > URCaps > Gripper > Dashboard`**，勾选 **`Gripper cable is connected to the wrist`**。这一步对应当前夹爪插在 UR7e 工具端的接线。
4. 打开 **`Installation > General > Tool I/O`**，在 **`I/O Interface Control > Controlled by`** 中选择 **`Robotiq_Grippers`**。工具供电和 RS485 交由这个 URCap 管理，核对工具电压为 **24 V**；不要同时交给其他 URCap 或自己的串口程序控制。
5. 回到 **`Installation > URCaps > Gripper > Dashboard`**，点击 **Scan**。看到夹爪图标后，清空手指之间的空间，点击 **Activate / Reactivate**；激活会产生夹爪运动。
6. 在同一页面勾选 **Show Gripper Toolbar**。按官方快速指南，可用 **E-Open / E-Close** 低速检查开合，再从顶部 **UR+** 工具栏检查夹爪控制。先确保示教器能正常开合夹爪，再运行 SDK。
7. **保存 Installation 文件**，后续加载同一个安装文件；否则重启后可能丢失 wrist 选项或工具 I/O 控制权。

工具端通信的核对值为 **115200 baud、8 data bits、1 stop bit、no parity**，通常由上述 URCap 设置，无需手填。若 `Controlled by` 下没有 `Robotiq_Grippers`，先检查 URCap 是否安装成功、是否重启，以及 wrist 复选框是否勾选；不要仅手动设 24 V 就继续运行。

### 3. 设置负载与 TCP

以下数值来自 [Robotiq UR 手册 §5.2.3](https://blog.robotiq.com/hubfs/support-files/2F-85_2F-140_UR_PDF_2026-03-17.pdf)，适用于其表中的 **标准 2F-85 + coupling，无腕部相机、力传感器、额外转接件或自制指尖，且未夹持物体**。如果实际安装不同，需要改成对应组件组合或标定值；不要使用 UMI 的 `1.81 kg` 和 `(2,-6,37) mm`。

1. 示教器打开 **`Installation > General > Payload`**，添加或选择当前工具负载，设置为当前使用的负载：
   - **Mass：`0.921 kg`**。
   - **Center of Gravity：CX = `0 mm`，CY = `0 mm`，CZ = `60 mm`**，相对工具法兰。
2. 打开 **`Installation > General > TCP`**，新建 `robotiq_2f85` TCP 并设为当前 TCP：
   - **X = `0 mm`，Y = `0 mm`，Z = `174 mm`**。
   - 本 SDK 示例让 TCP 坐标轴与法兰坐标轴平行，**RX = RY = RZ = `0`**。
   - 此设置用于示教器手动摆位时显示正确的工具位置。SDK 启动后会再次按 YAML 自动设置 TCP 和负载，因此 **只改示教器、不改 YAML 会被覆盖**。
3. 保存 Installation。核对 `configs/ur7e.yaml` 中已预填的对应参数：

```yaml
arms:
  - ip: 192.168.1.10
    commissioned: false
    tcp: [0, 0, 0.174, 0, 0, 0]
    payload_mass: 0.921
    payload_cog: [0, 0, 0.060]
```

这是配置摘录，保留原文件中的 `gripper`、`safety` 等字段。示教器的长度单位为 **mm**，YAML 为 **m**。更换指尖或转接件后，可在 TCP 页面用 **Measure** 向导标定新的 TCP，并同步回填 YAML；负载应包含所有末端附件，抓取物体后还需计入物体质量和质心。

### 4. 上电、摆位并切换远程控制

1. 确认急停已释放、现场允许启动。在示教器底部机器人状态/初始化页面点击 **ON**，再点击 **START** 松闸，等待机器人进入正常运行状态。
2. 在 Local Control 下，用 **Move** 页面或 Freedrive 将夹爪摆到空旷、可操作的位置。将 Move 页面的参考坐标系选为 **Base**，检查当前 TCP 位置位于配置的 `arms[0].safety.workspace_min / workspace_max` 内；默认范围是 `x=0.15..0.8 m`、`y=-0.5..0.5 m`、`z=0.1..0.8 m`。根据真实工作台设置范围，不要为了消除报错而无限扩大。
3. 打开 **`☰ > Settings > System > Remote Control`**，点击 **Enable**。
4. 再从右上角的控制模式菜单将 **Local Control 切换为 Remote Control**。只按 Enable 并没有真正切换到远程。
5. 若系统的 **`Settings > Security > Services`** 页面限制了接口，允许来自电脑的 **RTDE（30004）** 和 **Secondary Client Interface（30002）**。夹爪另需 **63352** 可达。

本 SDK 由 `ur-rtde` 上传控制脚本，**不需要额外安装 External Control URCap，也不需要在示教器点 Play 运行另一份运动程序**。此时停止其他控制客户端；Robotiq URCap 保持已安装并配置好的状态。

### 5. 在电脑检查并开始控制

```bash
conda activate ur7e_sdk
ur7e --config configs/ur7e.yaml diagnose
ur7e --config configs/ur7e.yaml gripper-status
```

- `diagnose` 应读出实际关节和 TCP；上电松闸后 `robot_mode` 应为 `7`，`safety_mode` 为 `1`（Normal）或 `2`（Reduced）。这是被动读取，不会开始伺服。
- `gripper-status` 应返回 **`STA=3`、`FLT=0`**；`POS` 随开合变化。它验证的是机器人上 **63352 的 Robotiq ASCII socket 服务**，不是裸 RS485。
- 若 `STA` 尚未激活，可清空夹持空间后运行 `ur7e --config configs/ur7e.yaml gripper-activate`。
- 若连接 63352 被拒绝或超时，回到第 2 步核对 URCap、wrist 选项及 Scan/Activate。若示教器能开合但端口仍不可用，需要确认当前 URCap 是否提供 Gello 使用的 63352 服务；本 SDK 不支持用其他端口上的 RS485 转发服务替代它。

以上检查通过、且已确认负载/TCP/工作空间后，将 YAML 的 **`arms[0].commissioned` 改为 `true`**，再运行：

```bash
ur7e --config configs/ur7e.yaml keyboard
```

按 **空格**启用，按一次 **w** 测试向基座 x 正方向移动 **2 mm**，按 **] / [** 测试夹爪，按 **q** 退出。不要用 `observe` 当作被动连通测试：它会启动完整伺服会话并保持当前位置。

夹爪配置中的 `open_raw / closed_raw` 默认 `0/255`，SDK 将位置线性换算为 `0..0.085 m` 的开度；这是估计宽度，不是精确测距。`speed / force` 为 Robotiq 原始 `0..255` 数值。

## 键盘与 ARX5

键盘模式需要交互终端。空格启用/暂停，`q`、Esc 或 Ctrl-C 退出；Tab 切换当前操作臂。每次按键移动一个小步，终端按住重复会继续步进；它不是检测按键释放的硬件使能开关。

| 按键 | 当前 UR 基座坐标系中的动作 |
| --- | --- |
| w / s | x 正 / 负，2 mm |
| a / d | y 正 / 负，2 mm |
| r / f | z 正 / 负，2 mm |
| i / k | 绕 x 正 / 负，0.01 rad |
| j / l | 绕 y 正 / 负，0.01 rad |
| u / o | 绕 z 正 / 负，0.01 rad |
| ] / [ | 打开 / 闭合，2 mm |

ARX5 模式需要另行安装 [ARX5 SDK](https://github.com/lanxinmob/arx5-sdk)，按照其说明编译 Python 绑定、配置 SocketCAN 和模型资源。以下命令在本仓库根目录执行，终端提示时输入 ARX5 SDK 的实际绝对路径。该目录的 `python/` 下应有编译好的 `arx5_interface*.so`，仅有 `.pyi` 文件不够。如果绑定依赖相对 URDF 路径，按 ARX5 SDK 要求设置工作目录，并用 `UR7E_SDK_DIR` 指定本项目配置的绝对路径。

```bash
conda activate ur7e_sdk
export UR7E_SDK_DIR="$PWD"
read -r -p "ARX5 SDK 目录（绝对路径）: " ARX5_SDK_DIR
export PYTHONPATH="$ARX5_SDK_DIR/python:${PYTHONPATH:-}"
python -c 'import arx5_interface; print("ARX5 import OK")' && \
ur7e --config "$UR7E_SDK_DIR/configs/ur7e.yaml" arx5
```

每个 UR 臂配置一个 `arx5` 条目。`model` / `interface` 对应 ARX 型号和 CAN 设备；`axes_rpy` 是 ARX 基座到 UR 基座的坐标轴旋转，单位为弧度；`position_scale` 控制平移比例；`gripper_width` 填主臂实际最大开度。`drag: true` 会开启 ARX 重力补偿、低阻尼拖动，不执行回零；结束时将主臂当前位置设置为保持目标并恢复原增益。

先单独检查 ARX 拖动手感（沿用上面的 Conda 和 `PYTHONPATH` 设置）：

```bash
ur7e --config configs/ur7e.yaml arx5-drag
```

该命令只连接 ARX，不连接或移动 UR；启动即进入拖动模式，q/Escape/Ctrl-C 退出。退出后 X5 SDK 会进入阻尼状态，不保证悬停，请托住主臂。普通 `arx5` 模式使用同一套拖动设置，暂停 UR 跟随时仍可拖动主臂。

在 `arx5` 条目中配置以下参数，默认与 ARX SDK `drag_mode.py` 一致：

```yaml
    drag: true
    gravity_scale: 1.0          # 重力前馈倍率，0 < 值 <= 2
    drag_kd_scale: 0.05         # 关节 kd = SDK default_kd * 此值
    drag_gripper_kd: 0.05
    gripper_open_readout: -3.4  # 夹爪全开电机读数；0 表示沿用模型值
```

程序在构造控制器前启用重力补偿和后台收发，清零当前位置命令的速度、附加力矩，再一次性设置关节/夹爪 `kp=0` 和低阻尼增益；拖动期间不持续发送位置命令，由 SDK 后台线程计算重力前馈。启动日志显示从控制器读回的重力向量和增益，设置未生效就退出。示例的自动回零未引入遥操作流程。

如果 SDK 示例能正常拖动，而遥操作出现抬头或下坠，先比较实际加载的模型。不同 Conda 环境、`PYTHONPATH` 和安装包可能使同名 `X5_umi` 加载不同 URDF；仅比较 `gravity_scale` 和增益不足以确认重力力矩一致。启动日志中的 `ARX binding`、`ARX URDF` 分别给出实际绑定和模型路径。可在对应条目显式指定 URDF，路径相对于主配置文件目录：

```yaml
  - model: X5_umi
    urdf_path: arx5/X5_umi_calibrated.urdf
    interface: can0
    drag: true
    gravity_scale: 1.0
```

这是模型选择示例，其余参数沿用前面的配置。提供的 `configs/arx5/X5_umi_calibrated.urdf` 将 `link6` 质量设为 0.802 kg，其他参数保持 ARX SDK 模型值；它适用于该负载配置，不是所有 X5/X5_umi 的通用标定。不要把它用于 `model: X5`，程序会检查 URDF 型号是否匹配。模型来自 [ARX5 SDK](https://github.com/lanxinmob/arx5-sdk)，许可证见 [configs/arx5/LICENSE](configs/arx5/LICENSE)。对自己的机械臂，应显式选用已验证的对应 URDF，而不是通过整体倍率掩盖某个连杆的质量误差。

如果仍然下坠，先核对模型、末端负载及安装方向；`gravity_vector` 可指定 ARX 基座系中的重力方向，单位 m/s²，默认沿用模型的 `[0, 0, -9.807]`，侧装或倒装必须据实际方向修改，不能用 `axes_rpy` 替代。模型匹配后，可小步调整 `gravity_scale`（例如下坠时从 1.0 调至 1.05，上浮时调至 0.95），每次重启生效。`drag_kd_scale` 只调运动阻尼，不能补偿重量。夹爪全开读数应按 ARX SDK 的 `calibrate.py` 标定；`-3.4` 是其拖动示例默认值，不保证适合所有批次。

首次按空格后，UR 使用 `moveJ` 缓慢回到记录的关节姿势，关节回位期间夹爪不发送开度命令，ARX 不主动移动。UR 到位并静止 0.2 秒后，程序读取当时 ARX 的夹爪开度快照，在 UR 位姿保持不动的情况下，以 `safety.max_gripper_speed` 的一半缓慢调整 UR 夹爪。对应开度按 `ARX 开度 / gripper_width * 0.085` 换算，默认两侧最大开度均为 0.085 m 时，目标宽度相同。夹爪达到 3 mm 误差内并保持 0.2 秒后，捕获两侧当前位姿和开度作为锚点，显示 `Enabled`，开始相对笛卡尔遥操作。同步夹爪期间请保持 ARX 开度不变，并确保 UR 夹爪内无障碍；到位失败或超时会退出。回位及夹爪同步过程中按 Space/q/Escape 或 Ctrl-C 会停止并退出；任何错误均不会继续进入遥操作。之后按空格暂停 UR，可重新摆放主臂，再启用时只重新建立锚点，不重复回位或开度同步。重新启动命令则会再次执行完整流程。

每个 `arx5` 条目配置对应 UR 的回位参数：

```yaml
    ur_home:
      reference: ur7e_reference.yaml  # 相对于当前主配置文件的目录
      speed: 0.1                    # 关节速度 rad/s
      acceleration: 0.1             # 关节加速度 rad/s^2
      timeout: 120.0                 # 单臂运动超时，秒
```

`configs/ur7e_reference.yaml` 保存了记录的六关节弧度值，顺序为肩部旋转、肩部抬升、肘部、腕部 1/2/3；`arm_index: 0` 对应第一个 UR 臂。换机器人或工位后，应使用 `ur7e --config configs/ur7e.yaml diagnose` 读取并核对目标关节值，更新参考文件。双臂需为第二个条目提供 `arm_index: 1` 的独立参考文件；程序先检查所有路径，再依次回位。键盘和模型模式不执行此回位。

开始回位前让 UR 静止并确认整臂运动空间无障碍。关节速度和加速度最多允许设为 0.2，速度还受 `safety.max_joint_speed` 限制。程序按最大关节间隔 0.02 rad 采样回位路径，检查 UR 关节安全条件和配置的 TCP 工作空间；运动期间监测状态新鲜度、速度、关节路径偏差及超时，并持续喂 UR watchdog。采样检查不是碰撞规划，TCP 的实际路径也不保证是直线；工作空间错误应通过调整起始姿势或核对工位范围解决。回位未完成时请勿拖动 UR。

遥操作平移使用相对位移，旋转将 ARX 的 xyz RPY 转为 SO(3) 相对旋转再映射到 UR 旋转向量；夹爪同样记录两侧开度锚点，UR 目标为其锚点开度加上 ARX 开度变化按两侧最大开度比例换算的增量，并限制在 0..0.085 m。启用前无需手动匹配两侧开度；暂停恢复会重新记录锚点，避免静止启用时发生夹爪目标跳变。主臂运动过快、反馈停止更新或进入 emergency 状态时退出并停止 UR。

若出现 `Waypoint deadline exceeds configured speed limits`，查看报错的 `translation`、`rotation` 或 `gripper`，分别代表平移、旋转或夹爪目标在剩余时间内需要的速度超限；它不等同于实测运动速度过快。日志包含 UR 编号、轨迹点编号、目标变化量、剩余时间、所需速度与限制值。

## Python 接口

进程使用 `spawn`，创建环境必须放在 `if __name__ == "__main__"` 保护下。下面是完整的短程移动示例，可先放入自己的应用脚本运行：

```python
import time
from ur7e_sdk import BimanualUREnv


def main():
    with BimanualUREnv("configs/sim.yaml") as env:
        obs = env.get_obs()
        action = env.action_from_obs(obs)   # (N, 7)
        action[:, 0] += 0.005               # 所有臂各向基座 x 方向移动 5 mm
        env.exec_action(action[None], [time.time() + 0.3])
        time.sleep(0.4)
        print(env.get_obs()["robot0_eef_pos"])


if __name__ == "__main__":
    main()
```

`get_obs(n_obs_steps=1)` 返回按 `obs_frequency` 间隔对齐的历史，`H=n_obs_steps`：

| 键 | 形状与含义 |
| --- | --- |
| `timestamp` | `(H,)`，对齐后的 Unix 秒 |
| `robot{i}_eef_pos` | `(H,3)`，各臂基座坐标系下 TCP 位置，米 |
| `robot{i}_eef_rot_axis_angle` | `(H,3)`，TCP 旋转向量，弧度，不是 RPY |
| `robot{i}_gripper_width` | `(H,1)`，夹爪开度，米 |
| `robot{i}_joint_pos` / `joint_vel` | `(H,6)`，弧度 / 弧度每秒 |
| `robot{i}_tcp_speed` | `(H,6)`，线速度 / 角速度 |
| `robot{i}_receive_timestamp` | `(1,)`，最新机械臂反馈的主机接收时间 |
| `robot{i}_gripper_receive_timestamp` | `(1,)`，最新夹爪反馈的主机接收时间 |
| `camera{i}_rgb`（可选） | `(H,height,width,3)`，uint8 RGB |
| `camera{i}_timestamp`（可选） | `(H,)`，实际选中帧的主机接收时间 |

历史不够时抛出 `RuntimeError`，等待积累后重试；不会复制首帧来伪造历史。SDK 使用主机接收时间，不宣称硬件曝光或设备时钟同步。位姿采用线性平移 + SO(3) Slerp，关节与宽度线性插值；相机取不晚于目标时刻的帧。对齐基准是所有传感器最新时间中的最早值。

`exec_action(actions, timestamps)`（别名 `exec_actions`）接收 `(T,N,7)` 或 `(T,7*N)` 的绝对动作；单动作也可以给 `(7*N,)`。每臂为 `[x,y,z,rx,ry,rz,width]`。时间戳必须是严格递增的 `(T,)` Unix 秒，最多 256 个点。返回入队的有效行数，**不表示运动完成**；执行中的异步故障在后续观测或关闭时抛出。

每批动作替换尚未执行的未来计划，并从接收时当前计划位姿连续衔接。截止时间不足两个控制周期的点会丢弃；整批过期返回 `0`，不刷新 watchdog。速度超限的截止时间会拒绝并停止，不擅自修改模型的执行时间。外部 Unix 时间使用会话固定偏移转换为单调时钟，运行中不要跳变主机系统时间。

## 模型控制与相机

自定义可信模块 `my_policy.py`，导出零参数工厂 `create_policy()`。工厂加载自己的模型并返回 `policy(obs)` 可调用对象。对象返回绝对动作数组 `(T,N,7)` / `(T,7*N)`，或者 `{"actions": ..., "timestamps": ...}`。将模型输出的归一化量、delta pose、6D rotation 或关节动作转换为上述接口后再返回。

```python
import numpy as np


def create_policy():
    # 在这里加载模型；此可运行示例仅保持观测到的位置。
    def policy(obs):
        action = np.concatenate([
            obs["robot0_eef_pos"][-1],
            obs["robot0_eef_rot_axis_angle"][-1],
            obs["robot0_gripper_width"][-1],
        ])
        return action[None, :]
    return policy
```

```bash
ur7e --config configs/sim.yaml policy --factory my_policy:create_policy \
  --rate 20 --obs-steps 2 --action-dt 0.05 --latency 0.2
```

不返回时间戳时，第一个目标时间为观测最后时间加 `--latency`，后续按 `--action-dt` 排列，推理耗时包含在这个预算内。模型应持续提交，默认两次命令间隔不得超过 1 秒，即使前一批还有未来点。推理长于预算、全部点过期时停止；可根据实测延迟配置预算和 `command_timeout`，不要用无限超时掩盖卡死。`--rate` 只是推理循环的频率上限，不改变控制进程频率。默认持位策略不包含模型权重或训练流程。

可选 OpenCV 相机：

```bash
python -m pip install -e '.[camera]'
```

在 YAML 顶层加入：

```yaml
cameras:
  - device: 0
    width: 640
    height: 480
    fps: 30
    history: 128
    timeout: 0.5
```

OpenCV 后端实际采集分辨率/帧率以设备支持为准。相机断流、过期或对齐缺口过大会使观测报错并停止，图像只在进程内历史中保留；本项目不提供录像、数据集或训练管理。

## 重要设计

```text
teleop/ 或 policy/ -> env/ -> control/ -> safety/ -> hardware/
                              |
                        shared_memory/
runtime/ 加载 configs/ 并管理上述生命周期
```

这些目录位于 `ur7e_sdk/`，配置位于根目录 `configs/`。一个独立进程负责整组机械臂，固定容量共享内存保存数值观测，有界队列传递完整动作批次；每个夹爪有独立通信线程。双臂伺服调用并发发出，仍不是硬件级同步。相机为可选独立采集线程。Python/Linux 循环不是硬实时系统，默认 125 Hz，可配置至 500 Hz，但必须根据实际 RTDE、网络和计算负载验证周期延迟。

双臂在 `arms` 中追加第二个独立 IP 与其 TCP、负载、安全配置，动作中按列表顺序对应 `robot0`、`robot1`；ARX5 模式也追加第二个主臂。各臂动作均在各自基座坐标系中，SDK 不隐式做双臂外参变换。任一臂校验失败或运行故障会触发整组停止。

独立 `safety/` 执行有限数值、TCP 工作空间、关节范围与速度、笛卡尔轨迹速度、夹爪目标速度、跟踪误差、状态新鲜度、动作期限与周期超时检查。硬件适配器检查 UR 安全模式和脚本运行状态，启动时查询初始位姿/IK 安全条件；运行中 IK 与控制器安全限制由 UR 自身执行，避免在伺服周期内插入同步查询。故障锁定当前会话，清除故障并检查现场后重新创建环境；不提供自动复位。输入队列拥塞也停止。

退出调用 `servoStop` / `stopScript`，夹爪停止请求为 `GTO=0`，不自动张开或闭合。UR 端设置 2 Hz watchdog，作为主机进程或通信失效后的补充保护；实际触发行为需要在对应控制器版本上验证。进程无法正常退出时会强制终止并报告异常，此时必须确认真实停止状态。

这是应用级安全层，不是安全认证系统，不能替代示教器配置、硬件急停和现场防护。当前不进行连杆/环境/双臂碰撞检测，不保证奇异点避让，也不进行加速度或 jerk 轨迹整形；TCP 在盒内不代表整臂无碰撞。初期双臂应配置互不干涉的物理工作区，并在 UR 自身配置安全平面、速度和力限制。保护停止只会上报，不会自动解除。

## 参考

- [Robotiq e-Series 快速安装指南](https://blog.robotiq.com/hubfs/support-files/Quick_start_2Finger_e-Series_nocropmarks_EN.pdf)：URCap 安装、wrist 选项、Scan/Activate。
- [Robotiq UR 完整手册](https://blog.robotiq.com/hubfs/support-files/2F-85_2F-140_UR_PDF_2026-03-17.pdf)：工具 I/O 控制权设置、§5.2.3 标准 2F-85 负载与 TCP。
- [UR PolyScope 5 Remote Control](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-usr-man/software/PolyScope/content/hamburger_menu_g5/System_remote_en.htm)：启用和切换远程控制的两个步骤。
- [UMI](https://github.com/real-stanford/universal_manipulation_interface)：`umi/real_world/rtde_interpolation_controller.py`、`umi/common/pose_trajectory_interpolator.py`、`umi/real_world/bimanual_umi_env.py` 的独立进程、时间戳动作和观测对齐设计。当前参考仓库的环境文件名为 `bimanual_umi_env`。
- [Gello](https://github.com/wuphilipp/gello_software)：`gello/robots/ur.py` 的 `URRobot` 与 `robotiq_gripper.py` 的 URCap socket 协议。此处针对 2F-85、米制宽度、有界等待和线程隔离重新实现。
- [ur-rtde](https://sdurobotics.gitlab.io/ur_rtde/)：RTDE 控制/接收接口、`servoL` 与 watchdog。
- [ARX5 SDK](https://github.com/lanxinmob/arx5-sdk)：`python/arx5_interface.pyi`、`python/examples/drag_mode.py` 的 Python API、RPY 位姿和重力补偿拖动配置。
