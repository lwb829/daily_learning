# import tensorflow as tf
#
# # 定义输入数据的形状
# input_shape = (None, 100)
#
# # 定义RNN层
# rnn_layer = tf.keras.layers.SimpleRNN(units=50, return_sequences=True, input_shape=input_shape)
#
# # 定义LSTM层
# lstm_layer = tf.keras.layers.LSTM(units=50, return_sequences=True, input_shape=input_shape)
#
# # 定义GRU层
# gru_layer = tf.keras.layers.GRU(units=50, return_sequences=True, input_shape=input_shape)
#
# # 构建模型
# model = tf.keras.Sequential([
#     tf.keras.layers.Embedding(input_dim=10000, output_dim=100, input_length=100),
#     rnn_layer,
#     lstm_layer,
#     gru_layer,
#     tf.keras.layers.Dense(units=1, activation='sigmoid')
# ])
#
# model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
# _*_coding:utf-8_*_

import tensorflow as tf
import numpy as np

'''
    TensorFlow中的RNN的API主要包括以下两个路径:
        1) tf.nn.rnn_cell(主要定义RNN的几种常见的cell)
        2) tf.nn(RNN中的辅助操作)
'''
# 一 RNN中的cell
# 基类(最顶级的父类): tf.nn.rnn_cell.RNNCell()
# 最基础的RNN的实现: tf.nn.rnn_cell.BasicRNNCell()
# 简单的LSTM cell实现: tf.nn.rnn_cell.BasicLSTMCell()
# 最常用的LSTM实现: tf.nn.rnn_cell.LSTMCell()
# RGU cell实现: tf.nn.rnn_cell.GRUCell()
# 多层RNN结构网络的实现: tf.nn.rnn_cell.MultiRNNCell()

# 创建cell
cell = tf.nn.rnn_cell.BasicRNNCell(num_units=128)
print(cell.state_size)
print(cell.output_size)

# shape=[4, 64]表示每次输入4个样本, 每个样本有64个特征
inputs = tf.placeholder(dtype=tf.float32, shape=[4, 64])

# 给定RNN的初始状态
s0 = cell.zero_state(4, tf.float32)
print(s0.get_shape())

# 对于t=1时刻传入输入和state0,获取结果值
output, s1 = cell.call(inputs, s0)
print(output.get_shape())
print(s1.get_shape())

# 定义LSTM cell
lstm_cell = tf.nn.rnn_cell.LSTMCell(num_units=128)
# shape=[4, 64]表示每次输入4个样本, 每个样本有64个特征
inputs = tf.placeholder(tf.float32, shape=[4, 48])
# 给定初始状态
s0 = lstm_cell.zero_state(4, tf.float32)
# 对于t=1时刻传入输入和state0,获取结果值
output, s1 = lstm_cell.call(inputs, s0)
print(output.get_shape())
print(s1.h.get_shape())
print(s1.c.get_shape())