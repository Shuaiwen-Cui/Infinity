# LINEAR NEURAL NETWORK - SINGLE LAYER PERCEPTRON

## What is a Linear Neural Network?

Linear Neural Network (LNN) is a basic neural network model, also known as a **single-layer perceptron**. It consists of an input layer, a linear transformation layer, and an output layer. The linear transformation layer is usually called a fully connected layer, where each input node is connected to each output node with a weighted connection.

In a linear neural network, each input feature is multiplied by a weight, then all results are summed and a bias is added. Finally, the result is processed by an activation function to generate the final output. Since the linear transformation is just a linear combination of the input, the model can only solve linearly separable problems.

The output of a linear neural network usually does not pass through an activation function, or passes through an identity function, so the output is a linear combination of the input. This makes linear neural networks work on some simple classification or regression problems, but perform poorly on nonlinear problems.

However, linear neural networks are often used as one of the basic components of more complex neural network models, such as a layer of a multilayer perceptron (MLP). In these models, a nonlinear activation function is usually added after the linear transformation layer to increase the model's expressive power, so that it can solve more complex problems.

## Useful Links

### Dive Into Deep Learning

<div class="grid cards" markdown>

-   :fontawesome-brands-square-github:{ .lg .middle } __Dive Into Deep Learning (EN) - CH04__

    --- 

    [:octicons-arrow-right-24: <a href="https://d2l.ai/chapter_linear-classification/index.html" target="_blank"> Portal </a>](#) 

-   :fontawesome-brands-square-github:{ .lg .middle } __Dive Into Deep Learning (ZH) - CH03__

    --- 

    [:octicons-arrow-right-24: <a href="https://zh.d2l.ai/chapter_linear-networks/index.html" target="_blank"> Portal </a>](#) 

</div>