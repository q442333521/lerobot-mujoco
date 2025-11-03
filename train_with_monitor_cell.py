"""
Training Cell with Real-time Monitor
带实时监控的训练单元格代码
直接复制此代码到Jupyter notebook的一个新cell中运行
"""

import subprocess
import re
from IPython.display import display, HTML, clear_output
import time

def train_with_monitor(config_path="smolvla_omy.yaml"):
    """
    Train model with real-time progress monitoring
    带实时进度监控的模型训练
    
    Args:
        config_path: Path to training configuration file
                    训练配置文件路径
    """
    # 启动训练进程 Start training process
    process = subprocess.Popen(
        ["python", "train_model.py", "--config_path", config_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1
    )
    
    # 初始化显示 Initialize display
    print("="*80)
    print("🚀 Training Started | 训练已开始")
    print("="*80)
    print()
    
    # 正则表达式匹配训练日志 Regex to match training logs
    step_pattern = re.compile(r'step:(\d+).*smpl:(\S+).*loss:([\d.]+).*lr:([\d.e-]+)')
    
    last_update_time = time.time()
    
    try:
        for line in process.stdout:
            # 每5秒更新一次进度显示（避免刷新太频繁）
            # Update progress display every 5 seconds (avoid too frequent refresh)
            current_time = time.time()
            
            # 匹配训练信息 Match training info
            match = step_pattern.search(line)
            if match and (current_time - last_update_time > 5):
                step = match.group(1)
                samples = match.group(2)
                loss = match.group(3)
                lr = match.group(4)
                
                # 清除之前的输出并显示新进度
                # Clear previous output and show new progress
                clear_output(wait=True)
                
                # 创建进度表格 Create progress table
                html_output = f"""
                <div style="border: 2px solid #4CAF50; padding: 20px; border-radius: 10px; 
                            background-color: #f9f9f9; font-family: monospace;">
                    <h2 style="color: #4CAF50; margin-top: 0;">
                        📊 Training Progress | 训练进度
                    </h2>
                    <table style="width: 100%; border-collapse: collapse;">
                        <tr style="background-color: #e8f5e9;">
                            <td style="padding: 10px; border: 1px solid #ddd; font-weight: bold;">
                                Step | 步数
                            </td>
                            <td style="padding: 10px; border: 1px solid #ddd; color: #2196F3; font-size: 18px;">
                                {step}
                            </td>
                        </tr>
                        <tr>
                            <td style="padding: 10px; border: 1px solid #ddd; font-weight: bold;">
                                Samples | 样本数
                            </td>
                            <td style="padding: 10px; border: 1px solid #ddd;">
                                {samples}
                            </td>
                        </tr>
                        <tr style="background-color: #fff3e0;">
                            <td style="padding: 10px; border: 1px solid #ddd; font-weight: bold;">
                                Loss | 损失值
                            </td>
                            <td style="padding: 10px; border: 1px solid #ddd; color: #FF5722; font-size: 18px;">
                                {loss}
                            </td>
                        </tr>
                        <tr>
                            <td style="padding: 10px; border: 1px solid #ddd; font-weight: bold;">
                                Learning Rate | 学习率
                            </td>
                            <td style="padding: 10px; border: 1px solid #ddd;">
                                {lr}
                            </td>
                        </tr>
                    </table>
                    <p style="margin-top: 15px; color: #666;">
                        💡 Tip: Loss should gradually decrease | 提示：损失值应逐渐下降
                    </p>
                </div>
                """
                
                display(HTML(html_output))
                last_update_time = current_time
            
            # 同时在下方显示完整日志 Also show full logs below
            if "INFO" in line or "WARNING" in line or "ERROR" in line:
                print(line, end='')
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Training interrupted by user | 用户中断训练")
        process.terminate()
        process.wait()
        return
    
    # 等待训练完成 Wait for training to finish
    process.wait()
    
    print("\n" + "="*80)
    print("✅ Training Completed! | 训练完成！")
    print("="*80)

# 使用方法 Usage:
# train_with_monitor("smolvla_omy.yaml")
