# MULTI LAYER PERCEPTRON

## What is a Multi Layer Perceptron?

A Multilayer Perceptron (MLP) is a machine learning model based on artificial neural networks, commonly used for solving classification and regression problems. It consists of multiple neural network layers, including at least one input layer, one or more hidden layers, and an output layer.

In an MLP, each neuron (also known as a node) is connected to all neurons in the previous layer, with each connection having a weight. Each neuron in the hidden layers and output layer includes an activation function, which is used to perform nonlinear transformations on input signals, thereby increasing the model's expressive power to learn and represent complex function relationships.

Training of MLPs typically involves using the backpropagation algorithm to adjust connection weights, aiming to minimize the error between predicted outputs and actual outputs. The backpropagation algorithm utilizes gradient descent to adjust weights along the negative gradient direction of the loss function, gradually optimizing the model.

MLPs find wide applications in various fields, including speech recognition, image recognition, natural language processing, recommendation systems, etc. Their flexibility and powerful nonlinear modeling capabilities make them one of the most commonly used models in the field of machine learning.

## Comparison with Single Layer Perceptron (Linear Neural Network)

Single Layer Perceptron and Multi Layer Perceptron are two different types of neural network models, with significant differences in structure and functionality. Here is a comparison between them:

1. **Structure**:
   - Single Layer Perceptron has only one neural network layer, including an input layer and an output layer.
   - Multi Layer Perceptron includes one or more hidden layers, in addition to the input layer and output layer.

2. **Nonlinear Capability**:
    - Single Layer Perceptron, due to having only a linear transformation layer, outputs a linear combination of inputs, and can only solve linearly separable problems.
    - Multi Layer Perceptron introduces nonlinear activation functions (such as ReLU, Sigmoid, tanh, etc.), enabling the network to learn and represent nonlinear relationships, and thus handle more complex data patterns and problems.

3. **Expressive Power**:
    - Single Layer Perceptron has limited expressive power and can only represent linearly separable functions.
    - Multi Layer Perceptron, with nonlinear activation functions and multiple hidden layers, can approximate any complex nonlinear function, and has stronger expressive power.

4. **Application Fields**:
    - Single Layer Perceptron is typically used for simple linear classification problems.
    - Multi Layer Perceptron is widely used in various fields, including image recognition, speech recognition, natural language processing, etc., for tasks that require handling complex data patterns.

5. **Training Algorithms**:
    - Single Layer Perceptron can be trained using simple online learning algorithms, such as the perceptron algorithm.
    - Multi Layer Perceptron typically uses gradient descent methods based on the backpropagation algorithm for training, which is more effective for networks with more complex multi-layer structures.

In summary, Single Layer Perceptron is suitable for simple linear problems, while Multi Layer Perceptron is suitable for more complex nonlinear problems, and is more common and effective in practical applications.

## Useful Links

### Dive Into Deep Learning

<div class="grid cards" markdown>

-   :fontawesome-brands-square-github:{ .lg .middle } __Dive Into Deep Learning (EN) - CH05__

    --- 

    [:octicons-arrow-right-24: <a href="https://d2l.ai/chapter_multilayer-perceptrons/index.html" target="_blank"> Portal </a>](#) 

-   :fontawesome-brands-square-github:{ .lg .middle } __Dive Into Deep Learning (ZH) - CH04__

    --- 

    [:octicons-arrow-right-24: <a href="https://zh.d2l.ai/chapter_multilayer-perceptrons/index.html" target="_blank"> Portal </a>](#) 

</div>