import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import os


def buildIKModel():
    checkpoint_path = "training/ik_1.ckpt"
    checkpoint_dir = os.path.dirname(checkpoint_path)
    model_ik = tf.keras.Sequential([
        layers.Dense(50, activation=tf.nn.relu, input_shape=[3]),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(50, activation=tf.nn.relu),
        layers.Dense(3)
    ])
    model_ik.compile(optimizer='adam', loss='mae')
    model_ik.summary()
    # Loads the weights
    model_ik.load_weights(checkpoint_path)
    return model_ik













