#!/bin/bash

ROOT=$HOME/data/shore_follower

ros2 run tensorflow_models_base train \
    --val_file $ROOT/val/labels.txt \
    --train_file $ROOT/train/labels.txt \
    --train_root $ROOT/train \
    --val_root $ROOT/val \
    --learning_rate 1.0000 \
    --batch_size 32 \
    --iter 10 \
    --dropout 0.1 \
    --classes 1 \
    --output $ROOT
