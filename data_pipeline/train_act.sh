#!/usr/bin/env bash
# Trains ACT on a dataset written by le2act.py.
#   ./train_act.sh pilot datasets/g1_box_reach      # 30k steps, for the catch->lift pilot (M3)
#   ./train_act.sh full  datasets/g1_box_act        # 100k steps, whole task (M4)
#   ./train_act.sh smoke datasets/fake_act cpu      # 20 steps on the CPU: checks the dataset loads and trains
set -euo pipefail
mode=${1:?pilot, full or smoke}
data=${2:?dataset root from le2act.py}
device=${3:-cuda}
name=act_g1_${mode}_$(basename "$data")

case $mode in
  pilot) steps=30000 batch=8 save=5000 ;;
  full) steps=100000 batch=8 save=10000 ;;
  smoke) steps=20 batch=2 save=20 ;;
  *) echo "mode must be pilot, full or smoke" >&2; exit 1 ;;
esac

# ACT paper defaults: chunk 50 (1 s at 50 Hz), KL weight 10, lr 1e-5, ResNet-18 per camera
lerobot-train \
  --dataset.repo_id=local/$(basename "$data") \
  --dataset.root="$data" \
  --policy.type=act \
  --policy.chunk_size=50 \
  --policy.n_action_steps=50 \
  --policy.kl_weight=10 \
  --policy.optimizer_lr=1e-5 \
  --policy.device="$device" \
  --policy.push_to_hub=false \
  --batch_size=$batch \
  --steps=$steps \
  --save_freq=$save \
  --log_freq=$(( steps < 200 ? 5 : 200 )) \
  --num_workers=$([ "$mode" = smoke ] && echo 0 || echo 4) \
  --output_dir=outputs/$name \
  --job_name=$name \
  --wandb.enable=false
