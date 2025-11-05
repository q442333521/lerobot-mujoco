#!/usr/bin/env python3
"""
Test script to fix the XML loading issue with absolute paths
"""

import os
import mujoco

# Set the working directory to the project root
os.chdir('/home/wzy/lerobot-mujoco')

# Use absolute path for the XML file
xml_path = '/home/wzy/lerobot-mujoco/01-Parol6/parol6_full.xml'

print(f"📖 加载MuJoCo模型 from: {xml_path}")
print(f"当前工作目录: {os.getcwd()}")

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
        
except Exception as e:
    print(f"❌ 模型加载失败: {e}")
    print("尝试检查文件路径和依赖资源...")