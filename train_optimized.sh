#!/bin/bash
# ===================================================
# Optimized Training Script with Tokenizer Fix
# 优化的训练脚本（修复tokenizer问题）
# ===================================================

echo "=================================================="
echo "🚀 Starting Optimized Training | 启动优化训练"
echo "=================================================="

# 1. Fix tokenizer parallelism warning
# 修复tokenizer并行化警告
export TOKENIZERS_PARALLELISM=false
echo "✓ TOKENIZERS_PARALLELISM=false"

# 2. Set optimal environment variables
# 设置最优环境变量
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
echo "✓ Thread optimization applied | 线程优化已应用"

# 3. Clear Python cache (optional but recommended)
# 清理Python缓存（可选但推荐）
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
echo "✓ Cache cleared | 缓存已清理"

# 4. Resume training from checkpoint
# 从检查点恢复训练
echo ""
echo "Starting training... | 开始训练..."
echo ""

python train_model.py --config_path smolvla_omy.yaml

echo ""
echo "=================================================="
echo "✅ Training Complete | 训练完成"
echo "=================================================="
