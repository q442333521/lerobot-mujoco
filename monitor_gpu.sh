#!/bin/bash
# ===================================================
# GPU and Training Monitor
# GPU和训练监控脚本
# ===================================================

echo "=================================================="
echo "📊 System Monitor | 系统监控"
echo "=================================================="
echo ""

while true; do
    clear
    echo "🖥️  GPU Status | GPU状态"
    echo "=================================================="
    nvidia-smi --query-gpu=index,name,utilization.gpu,memory.used,memory.total,temperature.gpu \
               --format=csv,noheader,nounits | \
    awk -F, '{printf "GPU %s: %s\n  Utilization: %s%%\n  Memory: %sMB / %sMB (%.1f%%)\n  Temp: %s°C\n\n", 
             $1, $2, $3, $4, $5, ($4/$5*100), $6}'
    
    echo "=================================================="
    echo "💾 Training Process | 训练进程"
    echo "=================================================="
    
    # 查找训练进程 Find training process
    train_pid=$(pgrep -f "train_model.py")
    if [ ! -z "$train_pid" ]; then
        echo "✓ Training is running | 训练正在进行"
        echo "  PID: $train_pid"
        
        # 显示CPU和内存使用 Show CPU and memory usage
        ps -p $train_pid -o %cpu,%mem,etime,cmd --no-headers | \
        awk '{printf "  CPU: %s%%\n  Memory: %s%%\n  Runtime: %s\n", $1, $2, $3}'
        
        # 显示最新的loss (从日志文件读取)
        if [ -f "ckpt/smolvla_omy/training.log" ]; then
            echo ""
            echo "📈 Latest Training Info | 最新训练信息"
            tail -n 1 ckpt/smolvla_omy/training.log 2>/dev/null || echo "  Log file not found"
        fi
    else
        echo "⚠️  No training process found | 未找到训练进程"
    fi
    
    echo ""
    echo "=================================================="
    echo "Press Ctrl+C to exit | 按 Ctrl+C 退出"
    echo "Refreshing in 5 seconds... | 5秒后刷新..."
    
    sleep 5
done
