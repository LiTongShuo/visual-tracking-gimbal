# Visual Tracking Gimbal

中文 | [English](README.en.md)

[![Python Verification](https://github.com/LiTongShuo-edu/visual-tracking-gimbal/actions/workflows/python.yml/badge.svg)](https://github.com/LiTongShuo-edu/visual-tracking-gimbal/actions/workflows/python.yml)

这是一个基于 Python / OpenCV 的双轴视觉追踪云台控制项目。系统运行在 Orange Pi 5 Pro 8G 上，从 720p USB 摄像头画面中识别靶标，计算目标相对画面中心的偏差和距离，并转换为两个 MG90S 舵机驱动的 3D 打印自制云台控制指令。

项目整理重点是把实机调试记录工程化为可公开验证的代码：控制逻辑可单元测试，硬件输出可替换为模拟控制器，视觉、控制和终端命令之间的共享状态通过锁保护。

## 工作流程

```text
摄像头画面
    |
OpenCV 灰度化、模糊、边缘检测、阈值分割
    |
轮廓层级筛选与靶标中心定位
    |
PnP 距离估计与误差计算
    |
线程安全共享状态
    |
YAW 软件 PWM + PITCH 视场角映射
    |
wiringpi 真实云台 / FakeGimbal 无硬件测试
```

## 功能模块

| 模块 | 作用 |
| --- | --- |
| `visual_tracking_gimbal/vision.py` | OpenCV 目标检测、轮廓点排序、PnP 距离估计 |
| `visual_tracking_gimbal/control.py` | YAW 软件 PWM、死区、限幅、PITCH 角度映射和平滑 |
| `visual_tracking_gimbal/state.py` | 线程安全共享状态，供视觉、控制和终端命令线程读写 |
| `visual_tracking_gimbal/hardware.py` | `wiringpi` 真实硬件控制与 `FakeGimbal` 模拟控制器 |
| `visual_tracking_gimbal/cli.py` | 终端命令解析和控制步进入口 |
| `tests/` | 无硬件自动化测试，覆盖控制、线程状态、CLI、视觉辅助函数 |

## 硬件环境

| 硬件 | 配置 |
| --- | --- |
| 主控 | Orange Pi 5 Pro 8G |
| 云台结构 | 3D 打印自制双轴云台 |
| 舵机 | MG90S 舵机 x 2 |
| 摄像头 | 720p USB 摄像头 |
| 控制接口 | Linux GPIO/PWM，代码中通过 `wiringpi` 封装硬件输出 |

## 控制策略

YAW 轴使用 MG90S 舵机驱动云台水平转动。为了避免小误差下长时间输出导致过冲，控制器将理想速度转换为软件 PWM：误差越大，占空比越高；目标进入中心死区时直接停止输出。

PITCH 轴同样使用 MG90S 舵机驱动俯仰角。整理后的逻辑不再持续累加控制量，而是将画面 Y 方向误差按视场角比例映射到目标角度，再通过平滑系数逐步逼近。目标置信度不足或舵机未使能时，PITCH 保持当前角度，YAW 输出 0。

## 安装

推荐使用 Python 3.12 虚拟环境：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

测试依赖：

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt
```

## 使用

查看命令行参数：

```powershell
.\.venv\Scripts\python.exe -m visual_tracking_gimbal --help
```

运行一个无摄像头、无硬件的控制演示：

```powershell
.\.venv\Scripts\python.exe -m visual_tracking_gimbal --demo-steps 20
```

进入交互式终端调试：

```powershell
.\.venv\Scripts\python.exe -m visual_tracking_gimbal
```

常用命令：

| 命令 | 作用 |
| --- | --- |
| `gui on` / `gui off` | 打开或关闭 GUI 状态标记 |
| `servo on` / `servo off` | 开启或关闭舵机输出状态 |
| `search` | 进入扫描模式 |
| `y` | 接受当前目标并回到追踪模式 |
| `n` | 拒绝当前目标并进入逃逸模式 |
| `status` | 打印当前共享状态 |
| `q` / `quit` / `exit` | 安全退出 |

真实硬件输出面向 Orange Pi 5 Pro 8G、720p USB 摄像头、MG90S 舵机 x 2、3D 打印自制云台和 `wiringpi` 环境：

```powershell
.\.venv\Scripts\python.exe -m visual_tracking_gimbal --hardware --yaw-pin 21 --pitch-pin 22
```

首次连接真实云台前，应先确认接线、电压范围、舵机方向和机械限位。

## 测试与验证

```powershell
.\.venv\Scripts\python.exe -m pytest
```

自动化测试覆盖：

- YAW 死区、限幅和软件 PWM 占空比。
- PITCH 角度限幅、视场角映射和平滑。
- 低置信度、目标丢失和舵机未使能时的冻结保护。
- `SharedState` 并发更新和线程安全快照。
- `FakeGimbal` 无硬件控制路径。
- CLI 命令解析。
- 视觉辅助函数、PnP 距离估计和合成图像靶标检测。

GitHub Actions 会在提交后运行同一组无硬件测试。

## 调试文档

- [调试与优化记录](docs/debug-notes.md)
- [简历项目条目参考](docs/resume-summary.md)
- [来源与公开边界](docs/source-boundary.md)

## 公开边界

本项目用于证明 Python、OpenCV、设备控制、多线程状态管理、调试优化和自动化测试实践。仓库不声明 gRPC/Protobuf、Appium 或 UIAutomation2 经验；这些能力需要由其他项目或后续补充材料单独支撑。

原始 Gemini HTML 导出文件和浏览器资源目录只作为本地整理素材保留，不作为公开项目核心内容提交。
