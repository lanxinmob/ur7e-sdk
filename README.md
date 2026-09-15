# UR7e SDK

UR7e + 工具端 Robotiq 2F-85 的 Python SDK。支持单臂、双臂、键盘、ARX5 主臂和模型动作序列；不依赖 ROS2、LeRobot、UMI 或 Gello 的安装。

## 安装与模拟运行

使用 Conda 管理环境。`environment.yml` 固定 Python 3.9，与本机现有 ARX5 二进制绑定一致，并安装 SDK 和 UR 硬件依赖。ARX5 模式的 Python 版本必须与 `arx5_interface` 编译版本一致。

```bash
cd ~/projects/ur7e_sdk
export http_proxy=http://127.0.0.1:7897
export https_proxy=http://127.0.0.1:7897
conda env create -f environment.yml
conda activate ur7e_sdk
ur7e observe
ur7e keyboard
ur7e policy --duration 5
ur7e --config configs/bimanual_sim.yaml policy --duration 5
```

默认 `configs/sim.yaml` 不连接机器人。`policy` 默认运行保持当前位姿的 `HoldPolicy`，用于检查完整调用链。模拟后端直接跟随目标，不模拟动力学、碰撞或接触。所有 CLI 命令在仓库根目录运行；其他目录使用 `--config` 的绝对路径。

以后使用时运行 `conda activate ur7e_sdk`。修改环境定义后，在仓库根目录运行 `conda env update -f environment.yml`。

## 连接 UR7e 与 2F-85

```bash
python -m pip install --proxy http://127.0.0.1:7897 -e '.[hardware]'
ur7e --config configs/ur7e.yaml diagnose
ur7e --config configs/ur7e.yaml gripper-status
```

1. 电脑与 UR 控制箱处于可直连的网络。修改 `configs/ur7e.yaml` 的 `ip`。`7897` 只用于依赖下载，RTDE 与夹爪通信直接访问机器人，不走 HTTP 代理。
2. 在示教器完成上电、松闸和远程控制设置；允许本机访问 RTDE `30004` 及 ur-rtde 上传脚本所用的 `30002`。确保没有另一个程序同时控制机器人。SDK 不自动松闸、不解除保护停止、不自动回零。
3. 按 Robotiq 对应接线和 URCap 手册配置工具端 **24 V**、工具通信和夹爪设备。必须有提供 `GET/SET` ASCII 协议的 Robotiq URCap socket 服务，默认机器人 IP 的 **63352**。工具端 RS485 接线本身不会创建这个网络服务；若端口不通，先检查 URCap、安装配置及服务状态。本 SDK 不实现裸 RS485/Modbus 转发。
4. `gripper-status` 应返回 `STA=3`、`FLT=0`。需要激活时运行下面的显式命令。激活可能使夹爪运动，先清空夹持空间；SDK 不自动执行全行程校准。
5. 填写实测 `tcp`、工具总质量 `payload_mass`、质心 `payload_cog`，根据实际工作台修改每臂 `safety`。TCP 平移单位为米、旋转为旋转向量；质心在工具法兰坐标系下以米表示。配置中的零值是占位值，不能直接当作 2F-85 的参数使用。
6. 核实配置后设置该臂 `commissioned: true`，从低速、空载开始运行。启动时实际 TCP 必须位于配置工作空间内，否则停止。`diagnose` 是被动 RTDE 读取；`observe` 会启动完整伺服会话、设置 TCP/负载并保持当前位置。

```bash
ur7e --config configs/ur7e.yaml gripper-activate
ur7e --config configs/ur7e.yaml observe
ur7e --config configs/ur7e.yaml keyboard
```

`open_raw` / `closed_raw` 是实测开/合原始位置，默认 `0/255`。SDK 对外宽度为 `0`（闭合）至 `0.085`（打开）米，内部线性换算；这只是位置反馈推算的宽度，接触物体时不是精确测距。`speed` 和 `force` 是 Robotiq 的 `0..255` 原始量，不是 m/s 或牛顿。

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

ARX5 模式使用本机 `~/projects/arx5-sdk`，先按照其说明编译 Python 绑定、配置 SocketCAN 和模型资源。在兼容的 Python 环境中安装本项目；如果绑定依赖相对 URDF 路径，按 ARX5 SDK 要求设置工作目录并传入本项目配置的绝对路径。

```bash
conda activate ur7e_sdk
export PYTHONPATH="$HOME/projects/arx5-sdk/python:${PYTHONPATH:-}"
python -c 'import arx5_interface'
ur7e --config "$HOME/projects/ur7e_sdk/configs/ur7e.yaml" arx5
```

每个 UR 臂配置一个 `arx5` 条目。`model` / `interface` 对应 ARX 型号和 CAN 设备；`axes_rpy` 是 ARX 基座到 UR 基座的坐标轴旋转，单位为弧度；`position_scale` 控制平移比例；`gripper_width` 填主臂实际最大开度。`drag: true` 会开启 ARX 重力补偿、低阻尼拖动，不执行回零；结束时将主臂当前位置设置为保持目标并恢复原增益。

先调整主臂姿势，再按空格捕获两侧当前位姿并启用映射。再次按空格暂停 UR，可重新摆放主臂，再启用时重新对齐。平移使用相对位移，旋转将 ARX 的 xyz RPY 转为 SO(3) 相对旋转再映射到 UR 旋转向量；夹爪使用归一化绝对开度，因此启用前应对齐开度。主臂运动过快、反馈停止更新或进入 emergency 状态时退出并停止 UR。

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
python -m pip install --proxy http://127.0.0.1:7897 -e '.[camera]'
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

- [UMI](https://github.com/real-stanford/universal_manipulation_interface)：`umi/real_world/rtde_interpolation_controller.py`、`umi/common/pose_trajectory_interpolator.py`、`umi/real_world/bimanual_umi_env.py` 的独立进程、时间戳动作和观测对齐设计。当前参考仓库的环境文件名为 `bimanual_umi_env`。
- [Gello](https://github.com/wuphilipp/gello_software)：`gello/robots/ur.py` 的 `URRobot` 与 `robotiq_gripper.py` 的 URCap socket 协议。此处针对 2F-85、米制宽度、有界等待和线程隔离重新实现。
- [ur-rtde](https://sdurobotics.gitlab.io/ur_rtde/)：RTDE 控制/接收接口、`servoL` 与 watchdog。
- 本机 `~/projects/arx5-sdk/python/arx5_interface.pyi`、`python/examples/drag_mode.py`：ARX5 Python API、RPY 位姿和重力补偿拖动配置。
