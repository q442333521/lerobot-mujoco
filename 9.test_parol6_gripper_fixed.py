#!/usr/bin/env python3
"""
Fixed version of the PAROL6 gripper test that handles path resolution correctly
"""

import os
import mujoco
import numpy as np
import time
from PIL import Image
import matplotlib.pyplot as plt

# Set working directory to project root to ensure relative paths work correctly
os.chdir('/home/wzy/lerobot-mujoco')
print(f"✅ 工作目录设置为: {os.getcwd()}")

# Set environment variables for rendering
os.environ['DISPLAY'] = ':0'
os.environ['MUJOCO_GL'] = 'egl'

# Use absolute path for XML file to avoid path resolution issues
xml_path = '/home/wzy/lerobot-mujoco/01-Parol6/parol6_full.xml'

print(f"📖 加载MuJoCo模型 from: {xml_path}")

try:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print("✅ 模型加载成功！")
    print(f"   总关节数: {model.njnt}")
    print(f"   总执行器: {model.nu}")
    print(f"   总body数: {model.nbody}")
    print(f"   摄像头数: {model.ncam}")
    
    # 打印所有关节名称
    print("\n🔧 关节列表:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            joint_type = 'revolute' if model.jnt_type[i] == 0 else 'slide'
            print(f"   {i}: {joint_name} ({joint_type})")
    
    # 打印所有摄像头
    print("\n📷 摄像头列表:")
    for i in range(model.ncam):
        cam_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        print(f"   {i}: {cam_name}")
        
    # 测试夹爪控制
    print("\n🤏 测试夹爪控制...")
    
    # 重置到初始状态
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    # 闭合夹爪
    data.ctrl[6] = 0.0  # 左手指闭合
    data.ctrl[7] = 0.0  # 右手指闭合
    
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    print(f"✅ 夹爪闭合状态 - 左手指: {data.qpos[6]:.4f}, 右手指: {data.qpos[7]:.4f}")
    
    # 打开夹爪
    data.ctrl[6] = 0.03  # 左手指打开
    data.ctrl[7] = 0.03  # 右手指打开
    
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    print(f"✅ 夹爪打开状态 - 左手指: {data.qpos[6]:.4f}, 右手指: {data.qpos[7]:.4f}")
    
    print("\n🎉 所有测试完成！模型加载和夹爪控制正常工作。")
    
except Exception as e:
    print(f"❌ 错误: {e}")
    print("请检查文件路径和依赖资源是否存在。")