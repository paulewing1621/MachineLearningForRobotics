#!/bin/bash

ROOT=$HOME/data/floor_plane

ros2 run tensorflow_models train \
    --val_file $ROOT/val/labels.txt \
    --train_file $ROOT/train/labels.txt \
    --train_root $ROOT/train \
    --val_root $ROOT/val \
    --learning_rate 1.000 \
    --batch_size 32 \
    --iter 10 \
    --dropout 0.1 \
    --classes 1 \
    --output $ROOT
