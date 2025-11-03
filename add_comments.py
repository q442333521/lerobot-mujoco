#!/usr/bin/env python3
import json
import sys

# 读取Jupyter Notebook文件
notebook_path = '/home/wzy/lerobot-mujoco/4.deploy.ipynb'
with open(notebook_path, 'r', encoding='utf-8') as f:
    notebook = json.load(f)

# 为策略加载部分添加中文注释
for cell in notebook['cells']:
    if cell['cell_type'] == 'code':
        # 策略加载部分
        if 'dataset_metadata = LeRobotDatasetMetadata(' in ''.join(cell['source']):
            new_source = [
                "# 1. 加载数据集元数据 - 包含了数据集的统计信息和特征描述\n",
                "dataset_metadata = LeRobotDatasetMetadata(\"omy_pnp\", root='./demo_data')  # 从demo_data目录加载omy_pnp数据集的元数据\n",
                "\n",
                "# 2. 特征处理 - 将数据集特征转换为策略可用的格式\n",
                "features = dataset_to_policy_features(dataset_metadata.features)  # 将数据集特征转换为策略可用的格式\n",
                "\n",
                "# 3. 区分输入和输出特征\n",
                "# 输出特征是机器人的动作（如关节角度）\n",
                "output_features = {key: ft for key, ft in features.items() if ft.type is FeatureType.ACTION}\n",
                "# 输入特征是观察值（如图像、机器人状态等）\n",
                "input_features = {key: ft for key, ft in features.items() if key not in output_features}\n",
                "# 移除手腕相机图像特征（本例中不使用）\n",
                "input_features.pop(\"observation.wrist_image\")\n",
                "\n",
                "# 4. 创建ACT策略配置\n",
                "# ACTConfig是Action Chunking with Transformers的配置类\n",
                "# chunk_size=10: 每次预测10个时间步的动作\n",
                "# n_action_steps=1: 每次执行1个动作步骤\n",
                "# temporal_ensemble_coeff=0.9: 时间集成系数，使动作预测更平滑\n",
                "cfg = ACTConfig(input_features=input_features, output_features=output_features, chunk_size= 10, n_action_steps=1, temporal_ensemble_coeff = 0.9)\n",
                "\n",
                "# 5. 处理时间戳信息\n",
                "delta_timestamps = resolve_delta_timestamps(cfg, dataset_metadata)\n",
                "\n",
                "# 6. 实例化策略模型并加载预训练权重\n",
                "# from_pretrained方法从本地目录加载预训练模型\n",
                "policy = ACTPolicy.from_pretrained('./ckpt/act_y', config = cfg, dataset_stats=dataset_metadata.stats)  # 从本地目录加载预训练模型\n",
                "\n",
                "# 7. 将策略模型移动到指定设备（GPU）\n",
                "policy.to(device)  # 将模型移到GPU上以加速计算\n"
            ]
            cell['source'] = new_source
        
        # 环境加载部分
        elif 'from mujoco_env.y_env import SimpleEnv' in ''.join(cell['source']):
            new_source = [
                "# 1. 导入MuJoCo环境模块\n",
                "from mujoco_env.y_env import SimpleEnv  # 导入简化的MuJoCo环境接口\n",
                "\n",
                "# 2. 指定场景XML文件路径\n",
                "xml_path = './asset/example_scene_y.xml'  # 场景描述文件，定义了机器人、物体和环境的物理属性\n",
                "\n",
                "# 3. 创建抓取放置环境实例\n",
                "# action_type='joint_angle'表示控制方式为关节角度控制（而非末端执行器位置控制）\n",
                "PnPEnv = SimpleEnv(xml_path, action_type='joint_angle')  # 创建Pick and Place环境\n"
            ]
            cell['source'] = new_source
        
        # 策略执行部分
        elif 'step = 0' in ''.join(cell['source']) and 'policy.reset()' in ''.join(cell['source']):
            new_source = [
                "# 1. 初始化环境和策略\n",
                "step = 0  # 初始化步数计数器\n",
                "PnPEnv.reset(seed=0)  # 重置环境，设置随机种子为0以确保可重复性\n",
                "policy.reset()  # 重置策略内部状态\n",
                "policy.eval()  # 将策略设置为评估模式（不进行梯度计算）\n",
                "\n",
                "# 2. 设置图像处理参数\n",
                "save_image = True  # 是否保存图像的标志\n",
                "img_transform = torchvision.transforms.ToTensor()  # 图像转换器，将PIL图像转换为PyTorch张量\n",
                "\n",
                "# 3. 主循环：只要MuJoCo查看器窗口保持打开状态就持续运行\n",
                "while PnPEnv.env.is_viewer_alive():\n",
                "    # 3.1 推进物理仿真一步\n",
                "    PnPEnv.step_env()\n",
                "    \n",
                "    # 3.2 以20Hz的频率执行控制逻辑（每秒20次）\n",
                "    if PnPEnv.env.loop_every(HZ=20):\n",
                "        # 3.3 检查任务是否完成\n",
                "        success = PnPEnv.check_success()\n",
                "        if success:\n",
                "            print('Success')  # 打印成功信息\n",
                "            # 重置环境和策略，准备下一轮任务\n",
                "            policy.reset()\n",
                "            PnPEnv.reset(seed=0)\n",
                "            step = 0\n",
                "            save_image = False\n",
                "            \n",
                "        # 3.4 获取当前环境状态\n",
                "        state = PnPEnv.get_ee_pose()  # 获取机器人末端执行器的位姿\n",
                "        \n",
                "        # 3.5 获取当前环境图像\n",
                "        image, wirst_image = PnPEnv.grab_image()  # 获取第三人称视角和手腕相机图像\n",
                "        # 处理第三人称视角图像\n",
                "        image = Image.fromarray(image)  # 将NumPy数组转换为PIL图像\n",
                "        image = image.resize((256, 256))  # 调整图像大小为256x256\n",
                "        image = img_transform(image)  # 转换为PyTorch张量\n",
                "        # 处理手腕相机图像\n",
                "        wrist_image = Image.fromarray(wirst_image)\n",
                "        wrist_image = wrist_image.resize((256, 256))\n",
                "        wrist_image = img_transform(wrist_image)\n",
                "        \n",
                "        # 3.6 构建输入数据字典\n",
                "        data = {\n",
                "            'observation.state': torch.tensor([state]).to(device),  # 机器人状态\n",
                "            'observation.image': image.unsqueeze(0).to(device),  # 第三人称视角图像\n",
                "            'observation.wrist_image': wrist_image.unsqueeze(0).to(device),  # 手腕相机图像\n",
                "            'task': ['Put mug cup on the plate'],  # 任务描述\n",
                "            'timestamp': torch.tensor([step/20]).to(device)  # 时间戳（秒）\n",
                "        }\n",
                "        \n",
                "        # 3.7 使用策略选择动作\n",
                "        action = policy.select_action(data)  # 根据当前观察选择动作\n",
                "        action = action[0].cpu().detach().numpy()  # 将动作张量转换为NumPy数组\n",
                "        \n",
                "        # 3.8 在环境中执行动作\n",
                "        _ = PnPEnv.step(action)  # 执行选定的动作\n",
                "        PnPEnv.render()  # 渲染环境\n",
                "        \n",
                "        # 3.9 更新步数计数器并检查任务完成情况\n",
                "        step += 1\n",
                "        success = PnPEnv.check_success()\n",
                "        if success:\n",
                "            print('Success')  # 打印成功信息\n",
                "            break  # 任务完成，退出循环\n"
            ]
            cell['source'] = new_source

# 保存修改后的Jupyter Notebook文件
with open(notebook_path, 'w', encoding='utf-8') as f:
    json.dump(notebook, f, ensure_ascii=False, indent=1)

print("已成功为4.deploy.ipynb添加详细中文注释！")