# 线性神经网络 - 单层感知机

## 什么是线性神经网络？

线性神经网络（Linear Neural Network）是一种基本的神经网络模型，也称为单层感知器（single-layer perceptron）。它由输入层、一个线性变换层和一个输出层组成。线性变换层通常被称为全连接层或密集层（fully connected layer），其中每个输入节点与输出节点之间都有权重连接。

在线性神经网络中，每个输入特征都与一个权重相乘，然后将所有结果相加，并添加一个偏置（bias）。最后，经过一个激活函数处理，生成最终的输出。由于线性变换只是对输入进行线性组合，因此该模型本质上只能解决线性可分的问题。

线性神经网络的输出通常不经过激活函数，或者经过恒等函数，因此输出是输入的线性组合。这使得线性神经网络在一些简单的分类或回归问题上能够工作，但在处理非线性问题时，表现不佳。

然而，线性神经网络通常被用作更复杂神经网络模型的基础组件之一，如多层感知器（MLP）的一层。在这些模型中，线性变换层后通常会接一个非线性的激活函数，以增加模型的表达能力，使其能够解决更复杂的问题。

## 资源链接

### 动手学深度学习

<div class="grid cards" markdown>

-   :fontawesome-brands-square-github:{ .lg .middle } __动手学深度学习（EN）- CH04__
    
    --- 
    
    [:octicons-arrow-right-24: <a href="https://d2l.ai/chapter_linear-classification/index.html" target="_blank"> 传送门 </a>](#)

-   :fontawesome-brands-square-github:{ .lg .middle } __动手学深度学习（ZH）- CH03__

    --- 
    
    [:octicons-arrow-right-24: <a href="https://zh.d2l.ai/chapter_linear-networks/index.html" target="_blank"> 传送门 </a>](#)

</div>