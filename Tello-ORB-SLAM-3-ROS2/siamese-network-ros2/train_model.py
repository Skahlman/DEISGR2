import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models, Input
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout

def create_base_network(input_shape):
    inputs = Input(shape=input_shape)
    x = Conv2D(32, (3,3), activation='relu')(inputs)
    x = MaxPooling2D()(x)
    x = Conv2D(64, (3,3), activation='relu')(x)
    x = MaxPooling2D()(x)
    x = Flatten()(x)
    x = Dense(128, activation='relu')(x)
    x = Dropout(0.1)(x)
    x = Dense(64, activation='relu')(x)
    model = models.Model(inputs, x, name="feature_extractor")
    return model

class L1DistanceLayer(layers.Layer):
    def __init__(self, **kwargs):
        super(L1DistanceLayer, self).__init__(**kwargs)

    def call(self, inputs):
        embedding_a, embedding_b = inputs
        return tf.math.abs(embedding_a - embedding_b)


def create_siamese_model(input_shape):
    base_network = create_base_network(input_shape)

    input_a = Input(shape=input_shape)
    input_b = Input(shape=input_shape)

    embedding_a = base_network(input_a)
    embedding_b = base_network(input_b)

    # Use custom distance layer
    distance = L1DistanceLayer()([embedding_a, embedding_b])
    outputs = Dense(1, activation='sigmoid')(distance)

    siamese_model = models.Model(inputs=[input_a, input_b], outputs=outputs)
    return siamese_model


num_pairs = 1000
img_height, img_width = 64, 64

X1 = np.random.rand(num_pairs, img_height, img_width, 1).astype('float32')
X2 = np.random.rand(num_pairs, img_height, img_width, 1).astype('float32')
y = np.random.randint(0, 2, size=(num_pairs, 1)).astype('float32')


siamese_model = create_siamese_model((img_height, img_width, 1))
siamese_model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
siamese_model.summary()

siamese_model.fit([X1, X2], y, batch_size=32, epochs=5, validation_split=0.2)


siamese_model.save('model.pb')
