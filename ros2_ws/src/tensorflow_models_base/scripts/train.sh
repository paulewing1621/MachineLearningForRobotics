#!/bin/bash


python3 train.py \
    --val_file /home/GTL/pewing/ros2_ws/src/tensorflow_models_base/validation2/labels.txt \
    --train_file /home/GTL/pewing/ros2_ws/src/tensorflow_models_base/training2/labels.txt \
    --train_root /home/GTL/pewing/ros2_ws/src/tensorflow_models_base/training2 \
    --val_root /home/GTL/pewing/ros2_ws/src/tensorflow_models_base/validation2 \
    --learning_rate 0.01 \
    --batch_size 32 \
    --iter 1000 \
    --dropout 0.1 \
    --classes 3 \
    --output /home/GTL/pewing/ros2_ws/src/tensorflow_models_base/output2
