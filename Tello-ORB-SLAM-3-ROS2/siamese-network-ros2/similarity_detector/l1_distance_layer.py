import tensorflow as tf
from tensorflow.keras.layers import Layer

class L1DistanceLayer(Layer):
    def __init__(self, **kwargs):
        super(L1DistanceLayer, self).__init__(**kwargs)

    def call(self, inputs):
        embedding_a, embedding_b = inputs
        return tf.math.abs(embedding_a - embedding_b)
