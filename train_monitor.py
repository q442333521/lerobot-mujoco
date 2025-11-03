#!/usr/bin/env python3
"""
Real-time Training Monitor Script
实时训练监控脚本
"""
import subprocess
import re
import sys
from datetime import datetime

def monitor_training(config_path="smolvla_omy.yaml"):
    """
    Monitor training progress in real-time
    实时监控训练进度
    """
    # 启动训练进程 Start training process
    process = subprocess.Popen(
        ["python", "train_model.py", "--config_path", config_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1
    )
    
    print("="*80)
    print("Training Started | 训练已开始")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*80)
    print()
    
    # 正则表达式匹配训练日志 Regular expression to match training logs
    step_pattern = re.compile(r'step:(\d+).*loss:([\d.]+)')
    
    try:
        for line in process.stdout:
            # 打印所有输出 Print all output
            print(line, end='')
            
            # 提取关键训练信息 Extract key training info
            match = step_pattern.search(line)
            if match:
                step = match.group(1)
                loss = match.group(2)
                
                # 实时显示进度 Real-time progress display
                print(f"\n{'='*60}")
                print(f"📊 PROGRESS | 训练进度")
                print(f"  Step | 步数: {step}")
                print(f"  Loss | 损失: {loss}")
                print(f"{'='*60}\n")
                sys.stdout.flush()
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Training interrupted by user | 用户中断训练")
        process.terminate()
        process.wait()
        return
    
    # 等待进程结束 Wait for process to finish
    process.wait()
    
    print("\n" + "="*80)
    print("✓ Training Completed | 训练完成")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*80)

if __name__ == "__main__":
    monitor_training()
