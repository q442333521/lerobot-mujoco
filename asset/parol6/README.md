# PAROL6 机械臂MuJoCo模型
# PAROL6 Robot Arm MuJoCo Model

## 📋 概述 | Overview

这是PAROL6六轴机械臂的完整MuJoCo模型，包含：
- 6自由度机械臂
- 2自由度平行夹爪（真实STL模型）
- 摄像头配置
- 物理参数和执行器

This is a complete MuJoCo model of the PAROL6 six-axis robot arm, including:
- 6-DOF robot arm
- 2-DOF parallel gripper (real STL models)
- Camera configurations
- Physical parameters and actuators

## 📁 文件结构 | File Structure

```
asset/parol6/
├── parol6.xml              # 主模型文件 | Main model file
├── README.md               # 本文档 | This document
└── meshes/                 # STL网格文件 | STL mesh files
    ├── base_link.STL       # 机械臂底座 (2.9MB)
    ├── L1.STL              # 关节1 (2.5MB)
    ├── L2.STL              # 关节2 (1.6MB)
    ├── L3.STL              # 关节3 (140KB)
    ├── L4.STL              # 关节4 (348KB)
    ├── L5.STL              # 关节5 (1.2MB)
    ├── L6.STL              # 关节6 (282KB)
    ├── gripper_base.stl    # 夹爪基座 (134KB)
    └── gripper_jaw.stl     # 夹爪手指 (251KB)
```

## 🔧 技术规格 | Technical Specifications

### 机械臂 | Robot Arm

| 参数 | 数值 |
|------|------|
| 自由度 | 6 DOF |
| 关节类型 | 旋转关节 (Hinge) |
| 控制类型 | 位置控制 (Position control) |
| 控制增益 | kp=3000 |
| 力矩范围 | ±1500 N |

### 关节范围 | Joint Ranges

| 关节 | 最小值 (rad) | 最大值 (rad) | 描述 |
|------|-------------|-------------|------|
| L1   | -1.70       | 1.70        | 基座旋转 |
| L2   | -0.98       | 1.00        | 肩部俯仰 |
| L3   | -2.00       | 1.30        | 肘部俯仰 |
| L4   | -2.00       | 2.00        | 腕部翻转 |
| L5   | -2.10       | 2.10        | 腕部俯仰 |
| L6   | 无限制      | 无限制       | 腕部旋转 |

### 夹爪 | Gripper

| 参数 | 数值 |
|------|------|
| 类型 | 2指平行夹爪 |
| 自由度 | 2 (左右手指独立) |
| 关节类型 | 滑动关节 (Slide) |
| 控制范围 | 0-0.03米 (每个手指) |
| 最大开口 | 0.06米 (总计) |
| 控制增益 | kp=1000 |

### 摄像头 | Cameras

1. **gripper_cam**: 夹爪视角摄像头
   - 位置: 夹爪附近
   - 视野: 60度
   - 用途: 第一人称视角，抓取操作

## 🚀 使用方法 | Usage

### 1. 独立使用parol6.xml

```python
import mujoco

# 加载模型
model = mujoco.MjModel.from_xml_path('asset/parol6/parol6.xml')
data = mujoco.MjData(model)

# 控制机械臂
data.ctrl[0] = 0.5   # L1关节
data.ctrl[1] = 0.3   # L2关节
# ... 其他关节

# 控制夹爪
data.ctrl[6] = 0.0   # 左手指闭合
data.ctrl[7] = 0.0   # 右手指闭合

# 运行仿真
mujoco.mj_step(model, data)
```

### 2. 在完整场景中使用

推荐使用 `example_scene_parol6.xml`，它包含：
- 天空和地面
- 桌子
- PAROL6机械臂
- 可交互物体（杯子、盘子）

```python
# 加载完整场景
model = mujoco.MjModel.from_xml_path('asset/example_scene_parol6.xml')
```

### 3. 在自定义场景中引入

```xml
<mujoco>
    <!-- 你的场景配置 -->

    <!-- 引入PAROL6 -->
    <include file="./parol6/parol6.xml" />

    <!-- 其他元素 -->
</mujoco>
```

## 🎨 可视化特性 | Visualization Features

### 彩色编码 | Color Coding

每个关节使用不同颜色便于识别：
- **base_link**: 浅灰色 (0.95, 0.95, 0.95)
- **L1**: 蓝色 (0.3, 0.3, 0.8)
- **L2**: 红色 (0.8, 0.3, 0.3)
- **L3**: 绿色 (0.3, 0.8, 0.3)
- **L4**: 黄色 (0.8, 0.8, 0.3)
- **L5**: 品红色 (0.8, 0.3, 0.8)
- **L6**: 青色 (0.3, 0.8, 0.8)
- **Gripper**: 灰色 (0.5, 0.5, 0.5)

这样可以在仿真中清晰地看到每个关节的运动。

## 🔄 坐标系 | Coordinate System

```
base_link (0, 0, 0.8715)
    │
    ├─ L1 (旋转 Z轴)
    │   └─ L2 (旋转 Z轴)
    │       └─ L3 (旋转 Z轴反向)
    │           └─ L4 (旋转 Z轴反向)
    │               └─ L5 (旋转 Z轴反向)
    │                   └─ L6 (旋转 Z轴反向, 无限制)
    │                       ├─ gripper_cam (0, -0.1, -0.05)
    │                       ├─ end_effector site (0, 0, -0.13)
    │                       └─ gripper_base (0, 0, -0.02)
    │                           ├─ gripper_left (0, 0.01, -0.11)
    │                           │   └─ 滑动 Y轴 [0, 0.03]
    │                           └─ gripper_right (0, -0.01, -0.11)
    │                               └─ 滑动 Y轴 [0, 0.03] (旋转180°)
    └─ tcp_link (末端工具中心)
```

## 📊 传感器 | Sensors

### 关节位置传感器 | Joint Position Sensors
- L1_pos, L2_pos, L3_pos, L4_pos, L5_pos, L6_pos
- rh_l1_pos, rh_r1_pos

### 关节速度传感器 | Joint Velocity Sensors
- L1_vel, L2_vel, L3_vel, L4_vel, L5_vel, L6_vel

## 🎮 控制接口 | Control Interface

### 执行器编号 | Actuator Indices

```python
# 机械臂关节 | Robot joints
data.ctrl[0]  # L1_motor
data.ctrl[1]  # L2_motor
data.ctrl[2]  # L3_motor
data.ctrl[3]  # L4_motor
data.ctrl[4]  # L5_motor
data.ctrl[5]  # L6_motor

# 夹爪 | Gripper
data.ctrl[6]  # rh_l1_motor (左手指)
data.ctrl[7]  # rh_r1_motor (右手指)
```

### 动作空间 | Action Space

- **维度**: 8D (6关节 + 2夹爪)
- **关节角度**: 弧度制 (radians)
- **夹爪开合**: 0-0.03米 (0=闭合, 0.03=完全打开)

## 🧪 测试和验证 | Testing and Validation

```bash
# 测试模型加载
cd /home/user/lerobot-mujoco/asset
python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('parol6/parol6.xml'); print('✅ 模型加载成功！')"

# 测试完整场景
python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('example_scene_parol6.xml'); print('✅ 场景加载成功！')"
```

## 📝 注意事项 | Notes

1. **文件路径**: STL文件使用相对路径，确保目录结构正确
2. **单位转换**: STL文件使用mm单位，通过scale="0.001"转换为m
3. **关节限制**: L6关节无限制旋转，适合连续旋转操作
4. **夹爪对称**: 右手指通过euler="0 0 3.14159"旋转180度实现对称

## 🔗 相关文件 | Related Files

- `example_scene_parol6.xml`: 完整场景文件
- `9.test_parol6_gripper.ipynb`: 夹爪测试Notebook
- `01-Parol6/02-urdf_to_mujoco_with_objects.py`: XML生成脚本
- `01-Parol6/GRIPPER_INTEGRATION_SUMMARY.md`: 夹爪集成总结

## 📊 版本历史 | Version History

| 版本 | 日期 | 描述 |
|------|------|------|
| 1.0.0 | 2025-11-05 | 初始版本，包含机械臂和夹爪 |
| 1.1.0 | 2025-11-05 | 添加彩色编码，修复显示问题 |

## 🙏 致谢 | Acknowledgments

- PAROL6机械臂设计: PAROL项目
- 夹爪模型: Rack & Pinion Robotic Gripper (Repaired Mesh)
- 参考实现: Robotis OMY MuJoCo模型

---

**维护者 | Maintainer:** Claude AI
**最后更新 | Last Updated:** 2025-11-05
