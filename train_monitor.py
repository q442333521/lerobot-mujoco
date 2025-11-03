#!/usr/bin/env python3
"""
Real-time Training Monitor Script with Auto Timestamped Output Directory
实时训练监控脚本（自动创建带时间戳的输出目录）
"""
import subprocess
import re
import sys
import yaml
import shutil
from datetime import datetime
from pathlib import Path

def create_timestamped_output_dir(base_config_path="smolvla_omy.yaml"):
    """
    Create a new config with timestamped output directory
    创建带时间戳输出目录的新配置文件
    
    Returns:
        new_config_path: Path to new config file
                        新配置文件的路径
    """
    # 生成时间戳 Generate timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 读取原始配置 Read original config
    with open(base_config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 获取原始输出目录 Get original output directory
    original_output_dir = config.get('output_dir', 'ckpt/smolvla_omy')
    base_dir = Path(original_output_dir).parent
    model_name = Path(original_output_dir).name
    
    # 创建新的带时间戳的输出目录 Create new timestamped output directory
    new_output_dir = f"{base_dir}/{model_name}_{timestamp}"
    config['output_dir'] = new_output_dir
    
    # 创建新配置文件 Create new config file
    new_config_path = f"smolvla_omy_{timestamp}.yaml"
    with open(new_config_path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
    
    print("="*80)
    print("📁 Output Directory Configuration | 输出目录配置")
    print("="*80)
    print(f"✓ Original config: {base_config_path}")
    print(f"✓ New config: {new_config_path}")
    print(f"✓ Original output dir: {original_output_dir}")
    print(f"✓ New output dir: {new_output_dir}")
    print("="*80)
    print()
    
    return new_config_path, new_output_dir

def monitor_training(config_path=None, auto_timestamp=True):
    """
    Monitor training progress in real-time
    实时监控训练进度
    
    Args:
        config_path: Path to config file. If None, use default
                    配置文件路径，如果为None则使用默认值
        auto_timestamp: Automatically create timestamped output directory
                       自动创建带时间戳的输出目录
    """
    # 如果启用自动时间戳，创建新配置
    # If auto timestamp is enabled, create new config
    if auto_timestamp:
        base_config = config_path or "smolvla_omy.yaml"
        config_path, output_dir = create_timestamped_output_dir(base_config)
    else:
        if config_path is None:
            config_path = "smolvla_omy.yaml"
    
    # 启动训练进程 Start training process
    cmd = ["python3", "train_model.py", "--config_path", config_path]
    
    print("="*80)
    print("🚀 Training Started | 训练已开始")
    print(f"📅 Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"⚙️  Config: {config_path}")
    if auto_timestamp:
        print(f"📂 Output: {output_dir}")
    print("="*80)
    print()
    
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1,
        env={**subprocess.os.environ, 'TOKENIZERS_PARALLELISM': 'false'}  # Fix tokenizer warning
    )
    
    # 正则表达式匹配训练日志 Regular expression to match training logs
    step_pattern = re.compile(r'step:(\d+).*smpl:(\S+).*ep:(\d+).*epch:([\d.]+).*loss:([\d.]+).*lr:([\S]+)')
    
    try:
        for line in process.stdout:
            # 打印所有输出 Print all output
            print(line, end='')
            
            # 提取关键训练信息 Extract key training info
            match = step_pattern.search(line)
            if match:
                step = match.group(1)
                samples = match.group(2)
                episodes = match.group(3)
                epochs = match.group(4)
                loss = match.group(5)
                lr = match.group(6)
                
                # 实时显示进度 Real-time progress display
                print(f"\n{'='*80}")
                print(f"📊 TRAINING PROGRESS | 训练进度")
                print(f"{'='*80}")
                print(f"  Step | 步数:          {step}/2000 ({float(step)/20:.1f}%)")
                print(f"  Samples | 样本数:     {samples}")
                print(f"  Episodes | 回合数:    {episodes}")
                print(f"  Epochs | 轮数:        {epochs}")
                print(f"  Loss | 损失:         {loss}")
                print(f"  Learning Rate | 学习率: {lr}")
                print(f"{'='*80}\n")
                sys.stdout.flush()
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Training interrupted by user | 用户中断训练")
        print("Saving checkpoint... | 正在保存检查点...")
        process.terminate()
        process.wait()
        return
    
    # 等待进程结束 Wait for process to finish
    process.wait()
    
    print("\n" + "="*80)
    print("✅ Training Completed! | 训练完成！")
    print(f"📅 Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    if auto_timestamp:
        print(f"📂 Results saved to: {output_dir}")
    print("="*80)

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Train model with monitoring | 带监控的模型训练')
    parser.add_argument('--config', type=str, default="smolvla_omy.yaml",
                       help='Config file path | 配置文件路径')
    parser.add_argument('--no-timestamp', action='store_true',
                       help='Disable auto timestamp | 禁用自动时间戳')
    
    args = parser.parse_args()
    
    monitor_training(
        config_path=args.config,
        auto_timestamp=not args.no_timestamp
    )
