import tensorflow as tf
from tensorflow.keras import Model
import tensorflow.keras.layers as tfkl
import keras

# Made for TensorFlow 2.6.X

# This code implements a Convolutional Neural Network with a tensorboard front-end. Tensorboard
# is a nice visualization tool embedded within TensorFlow.
# Please note that some errors have been intentionaly made inside this code. They can be found in the layers connections or layer size.

def create_cnn_model(width,height,channel,classes, mean, std, drop):
    return keras.Sequential([
        tfkl.Input(shape=(height,width,channel)),
        tfkl.Normalization(axis=-1,mean=mean,variance=std**2, name='g_norm'),
        tfkl.Conv2D(3, [6,6], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_1'),
        tfkl.LayerNormalization(name='layer_norm_1'),
        tfkl.Conv2D(6, [5,5], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_2'),
        tfkl.LayerNormalization(name='layer_norm_2'),
        tfkl.Conv2D(12, [4,4], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_3'),
        tfkl.Flatten(),
        tfkl.Dense(100, name='dense'),
        tfkl.Dropout(drop, name='dropout'),
        tfkl.Dense(classes, activation=keras.activations.sigmoid, name='raw_output'),

    ])


if __name__ == '__main__':
    import numpy as np
    m = create_cnn_model(32,32,3,2,0,1,0.1)
    print(m(np.zeros([1,32,32,3])))
    m.save("model.keras")
    tm=tf.keras.models.load_model("model.keras")
    print(tm(np.zeros([1,32,32,3])))


