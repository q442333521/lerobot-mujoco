#!/usr/bin/env python3
"""
测试新的XML文件是否能正确显示机械臂主体
"""

import mujoco
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

def render_camera(model, data, camera_name="agentview", width=640, height=480):
    """从指定摄像头渲染图像"""
    renderer = mujoco.Renderer(model, height=height, width=width)
    renderer.update_scene(data, camera=camera_name)
    image = renderer.render()
    return image

def test_new_xml():
    """测试新的XML文件"""
    print("🚀 开始测试新的XML文件...")
    
    # 使用新的修复版XML文件
    xml_path = '/home/wzy/lerobot-mujoco/asset/example_scene_parol6.xml'
    
    print(f"📖 加载XML文件: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print("✅ 模型加载成功！")
    print(f"   总关节数: {model.njnt}")
    print(f"   总执行器: {model.nu}")
    print(f"   总body数: {model.nbody}")
    print(f"   摄像头数: {model.ncam}")
    
    # 检查所有body
    print("\n🔍 检查所有body:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name:
            print(f"   Body {i}: {body_name}")
    
    # 检查关键body是否存在
    expected_bodies = ['base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'gripper_base']
    missing_bodies = []
    for body_name in expected_bodies:
        try:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id == -1:
                missing_bodies.append(body_name)
        except:
            missing_bodies.append(body_name)
    
    if not missing_bodies:
        print("✅ 所有关键body都存在！")
    else:
        print(f"⚠️ 缺失的body: {missing_bodies}")
    
    # 重置到初始状态
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    # 创建输出目录
    output_dir = '/home/wzy/lerobot-mujoco/test_img_output'
    os.makedirs(output_dir, exist_ok=True)
    
    # 渲染多个视角
    camera_names = ['agentview', 'gripper_cam', 'topview', 'sideview']
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    print(f"\n📸 渲染多视角图像...")
    for idx, cam_name in enumerate(camera_names):
        try:
            image = render_camera(model, data, camera_name=cam_name)
            ax = axes[idx // 2, idx % 2]
            ax.imshow(image)
            ax.set_title(f'{cam_name}', fontsize=14, fontweight='bold')
            ax.axis('off')
            
            # 保存单个图像
            img_path = os.path.join(output_dir, f'test_{cam_name}.png')
            img_pil = Image.fromarray(image)
            img_pil.save(img_path)
            print(f"   💾 保存 {cam_name} 视角: {img_path}")
            
        except Exception as e:
            print(f"⚠️ 无法渲染摄像头 {cam_name}: {e}")
    
    plt.tight_layout()
    
    # 保存多视角组合图
    combined_path = os.path.join(output_dir, 'test_multi_view_combined.png')
    plt.savefig(combined_path, dpi=150, bbox_inches='tight')
    print(f"💾 保存多视角组合图: {combined_path}")
    
    plt.show()
    
    # 测试夹爪控制
    print(f"\n🤏 测试夹爪控制...")
    
    # 测试闭合
    data.ctrl[6] = 0.0  # 左手指闭合
    data.ctrl[7] = 0.0  # 右手指闭合
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    closed_image = render_camera(model, data, camera_name='gripper_cam')
    closed_path = os.path.join(output_dir, 'test_gripper_closed.png')
    Image.fromarray(closed_image).save(closed_path)
    print(f"   💾 保存夹爪闭合状态: {closed_path}")
    
    # 重置并测试打开
    mujoco.mj_resetData(model, data)
    data.ctrl[6] = 0.03  # 左手指完全打开
    data.ctrl[7] = 0.03  # 右手指完全打开
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    open_image = render_camera(model, data, camera_name='gripper_cam')
    open_path = os.path.join(output_dir, 'test_gripper_open.png')
    Image.fromarray(open_image).save(open_path)
    print(f"   💾 保存夹爪打开状态: {open_path}")
    
    print(f"\n🎉 测试完成！")
    print(f"📁 所有图片已保存到: {output_dir}")
    print(f"\\💡 现在请检查保存的图片，确认机械臂主体是否正确显示")

if __name__ == "__main__":
    test_new_xml()