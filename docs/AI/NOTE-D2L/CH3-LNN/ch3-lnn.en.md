# CH03 - Linear Neural Networks
Before we worry about making our neural networks deep, it will be helpful to implement some shallow ones, for which the inputs connect directly to the outputs. This will prove important for a few reasons.
First, rather than getting distracted by complicated architectures, we can focus on the basics of neural network training, including parametrizing the output layer, handling data, specifying a loss function, and training the model.
Second, this class of shallow networks happens to comprise the set of linear models, which subsumes many classical methods of statistical prediction, including linear and softmax regression. Understanding these classical tools is pivotal because they are widely used in many contexts and we will often need to use them as baselines when justifying the use of fancier architectures. This chapter will focus narrowly on linear regression and the next one will extend our modeling repertoire by developing linear neural networks for classification.

## 3.1. Linear Regression
### 3.1.1. Basics
Regression problems pop up whenever we want to predict a numerical value. 

#### 3.1.1.1 Models
At the heart of every solution is a model that describes how features can be transformed into an estimate of the target. The assumption of linearity means that the expected value of the target (price) can be expressed as a weighted sum of the features (area and age):

$$\mathbb{E}[\text{price}|\text{area,age}] = w_{\text{area}} \cdot \text{area} + w_{\text{age}} \cdot \text{age} + b$$

Here, $w_{\text{area}}$ and $w_{\text{age}}$ are weights that determine the influence of each feature on our prediction and $b$ is the bias term, which offsets any constant factors that would shift our prediction away from the true price.

Strictly speaking, the formular is an affine transformation of input features, which is characterized by a linear transformation of features via a weighted sum, combined with a translation via the added bias. Given a dataset, our goal is to choose the weights $w_{\text{area}}$ and $w_{\text{age}}$ and the bias $b$ such that on average, the predictions actually made by our model are as close as possible to the true prices observed in the dataset.

#### 3.1.1.2 Loss Functions
Naturally, fitting our model to the data requires that we agree on some measure of fitness (or, equivalently, of unfitness). Loss functions quantify the distance between the real and predicted values of the target. The loss will usually be a nonnegative number where smaller values are better and perfect predictions incur a loss of 0. For regression problems, the most common loss function is the squared error. When our prediction for an example $i$ is $\hat{y}^{(i)}$ and the corresponding true label is $y^{(i)}$, the squared error is given by:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{2} \left(\hat{y}^{(i)} - y^{(i)}\right)^2$$

Note that large differences between estimate and observation are penalized much more than small ones due to the quadratic dependence. To measure the quality of a model on the entire dataset, we simply average (or equivalently, sum) the losses on the training set. To measure the quality of a model on the entire dataset of `n` examples, we simply average (or equivalently, sum) the losses on the training set:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{n} \sum_{i=1}^n \left(\hat{y}^{(i)} - y^{(i)}\right)^2$$

When training the model, we seek parameters $\mathbf{w}^*,b^*$ that minimize the total loss across all training examples:

$$\mathbf{w}^*,b^* = \arg \min_{\mathbf{w},b} \mathcal{L}(\mathbf{w},b)$$

#### 3.1.1.3 Analytic Solution

Unlike most of the models that we will cover, linear regression presents us with a surprisingly easy optimization problem. In particular, we can find the optimal parameters (as assessed on the training data) analytically by applying a simple formula as follows. First, we can collect all the data examples into a matrix $\mathbf{X}$ and collect all the corresponding target values in a vector $\mathbf{y}$. Using linear algebra, we can express the squared error loss for all examples as:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{2} (\mathbf{X} \mathbf{w} + b \mathbf{1} - \mathbf{y})^\top (\mathbf{X} \mathbf{w} + b \mathbf{1} - \mathbf{y})$$

where we have used the fact that the vector of all ones $\mathbf{1}$ satisfies $\mathbf{X}^\top \mathbf{1} = \mathbf{0}$. Expanding out the squared terms and reorganizing terms, we get:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{2} \left(\mathbf{w}^\top \mathbf{X}^\top \mathbf{X} \mathbf{w} + b^2 \mathbf{1}^\top \mathbf{1} + \mathbf{y}^\top \mathbf{y} - 2 \mathbf{w}^\top \mathbf{X}^\top \mathbf{y} - 2 b \mathbf{1}^\top \mathbf{X} \mathbf{w} + 2 b \mathbf{1}^\top \mathbf{y} \right)$$

Since the first three terms are constant with respect to $\mathbf{w}$ and $b$, minimizing the loss is equivalent to minimizing:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{2} \left(\mathbf{w}^\top \mathbf{X}^\top \mathbf{X} \mathbf{w} - 2 \mathbf{w}^\top \mathbf{X}^\top \mathbf{y} + b \mathbf{1}^\top \mathbf{1} + 2 b \mathbf{1}^\top \mathbf{y} \right)$$

Since the first three terms are constant with respect to $\mathbf{w}$ and $b$, minimizing the loss is equivalent to minimizing:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{2} \left(\mathbf{w}^\top \mathbf{X}^\top \mathbf{X} \mathbf{w} - 2 \mathbf{w}^\top \mathbf{X}^\top \mathbf{y} + b \mathbf{1}^\top \mathbf{1} + 2 b \mathbf{1}^\top \mathbf{y} \right)$$

Note that the first term does not depend on $b$ and the second term does not depend on $\mathbf{w}$. Dropping these terms leaves us with the optimization problem of minimizing:

$$\mathcal{L}(\mathbf{w},b) = \frac{1}{2} \left(\mathbf{w}^\top \mathbf{X}^\top \mathbf{X} \mathbf{w} - 2 \mathbf{w}^\top \mathbf{X}^\top \mathbf{y} \right)$$

Recall that $\mathbf{X}^\top \mathbf{X}$ is a Gram matrix. It is guaranteed to be positive semidefinite, which implies that $\mathbf{w}^\top \mathbf{X}^\top \mathbf{X} \mathbf{w} \geq 0$. Consequently, the quadratic term is always nonnegative and the problem is convex. Setting the gradient equal to zero yields the analytic solution:

$$\mathbf{w}^* = (\mathbf{X}^\top \mathbf{X})^{-1} \mathbf{X}^\top \mathbf{y}$$

This is a system of $d$ linear equations in $d$ unknowns. Assuming that the inverse $(\mathbf{X}^\top \mathbf{X})^{-1}$ exists, it follows that the unique global minimum is achieved when:

$$\mathbf{w}^* = (\mathbf{X}^\top \mathbf{X})^{-1} \mathbf{X}^\top \mathbf{y}$$

#### 3.1.1.4 Minibatch Stochastic Gradient Descent

Fortunately, even in cases where we cannot solve the models analytically, we can still often train models effectively in practice. Moreover, for many tasks, those hard-to-optimize models turn out to be so much better that figuring out how to train them ends up being well worth the trouble.

The key technique for optimizing nearly every deep learning model, and which we will call upon throughout this book, consists of **iteratively reducing the error by updating the parameters in the direction that incrementally lowers the loss function.** This algorithm is called **gradient descent**. 

The most naive application of gradient descent consists of taking the derivative of the loss function, which is an average of the losses computed on every single example in the dataset. In practice, this can be extremely slow: we must pass over the entire dataset before making a single update, even if the update steps might be very powerful. Even worse, if there is a lot of redundancy in the training data, the benefit of a full update is limited.

The other extreme is to consider only a single example at a time and to take update steps based on one observation at a time. The resulting algorithm, stochastic gradient descent (SGD) can be an effective strategy, even for large datasets. Unfortunately, SGD has drawbacks, both computational and statistical. One problem arises from the fact that processors are a lot faster multiplying and adding numbers than they are at moving data from main memory to processor cache. It is up to an order of magnitude more efficient to perform a matrix–vector multiplication than a corresponding number of vector–vector operations. This means that it can take a lot longer to process one sample at a time compared to a full batch. A second problem is that some of the layers, such as batch normalization, only work well when we have access to more than one observation at a time.

The solution to both problems is to pick an **intermediate strategy**: rather than taking a full batch or only a single sample at a time, we take a minibatch of observations (Li et al., 2014). The specific choice of the size of the said minibatch depends on many factors, such as the amount of memory, the number of accelerators, the choice of layers, and the total dataset size. Despite all that, a number between 32 and 256, preferably a multiple of a large power of 2, is a good start. This leads us to minibatch stochastic gradient descent.

In its most basic form, in each iteration $t$ we first randomly sample a minibatch $\mathcal{B}^{(t)}$ consisting of a fixed number of training examples. We then compute the derivative (gradient) of the average loss on the minibatch (averaging loss over the minibatch examples) with regard to the model parameters. Finally, we multiply the gradient by a predetermined positive value $\eta$ and subtract the resulting term from the current parameter values. Next, we iterate over the dataset again and again until we find an answer that is good enough (or until we run out of time).

$$\mathbf{w}^{(t+1)} \leftarrow \mathbf{w}^{(t)} - \eta \frac{1}{|\mathcal{B}^{(t)}|} \sum_{i \in \mathcal{B}^{(t)}} \nabla_{\mathbf{w}} \mathcal{L}^{(i)} (\mathbf{w}^{(t)})$$

In summary, minibatch SGD proceeds as follows: (i) initialize the values of the model parameters, typically at random; (ii) iteratively sample random minibatches from the data, updating the parameters in the direction of the negative gradient. For quadratic losses and affine transformations, this has a closed-form expansion:

$$\mathbf{w}^{(t+1)} \leftarrow (\mathbf{X}^\top \mathbf{X})^{-1} \mathbf{X}^\top \mathbf{y}$$

Frequently minibatch size and learning rate are user-defined. Such tunable parameters that are not updated in the training loop are called hyperparameters. They can be tuned automatically by a number of techniques, such as **Bayesian optimization**. In the end, the quality of the solution is typically assessed on a separate validation dataset (or validation set).

#### 3.1.1.5 Gradient Descent for Linear Regression
Given the model $\hat{y} = \mathbf{w}^\top \mathbf{x} + b$ we can now make predictions for a new example, e.g., predicting the sales price of a previously unseen house given its area $x_{1}$ and age $x_{2}$. Deep learning practitioners have taken to calling the prediction phase inference but this is a bit of a misnomer—inference refers broadly to any conclusion reached on the basis of evidence, including both the values of the parameters and the likely label for an unseen instance. If anything, in the statistics literature inference more often denotes parameter inference and this overloading of terminology creates unnecessary confusion when deep learning practitioners talk to statisticians. In the following we will stick to prediction whenever possible.

### 3.1.2. Vectorization for Speed
When training our models, we typically want to process whole minibatches of examples simultaneously. Doing this efficiently requires that we vectorize the calculations and leverage fast linear algebra libraries rather than writing costly for-loops in Python.

To see why this matters so much, let’s consider two methods for adding vectors. To start, we instantiate two 10,000-dimensional vectors containing all 1s. In the first method, we loop over the vectors with a Python for-loop. In the second, we rely on a single call to +.

### 3.1.3. The Normal Distribution and Squared Loss

### 3.1.4. Linear Regression as a Neural Network

### 3.1.5. Summary
In this section, we introduced traditional linear regression, where the parameters of a linear function are chosen to minimize squared loss on the training set. We also motivated this choice of objective both via some practical considerations and through an interpretation of linear regression as maximimum likelihood estimation under an assumption of linearity and Gaussian noise. After discussing both computational considerations and connections to statistics, we showed how such linear models could be expressed as simple neural networks where the inputs are directly wired to the output(s). While we will soon move past linear models altogether, they are sufficient to introduce most of the components that all of our models require: parametric forms, differentiable objectives, optimization via minibatch stochastic gradient descent, and ultimately, evaluation on previously unseen data.

## 3.2. Object-Oriented Design for Implementation
In our introduction to linear regression, we walked through various components including the **data**, the **model**, the **loss function**, and the **optimization algorithm**. Indeed, linear regression is one of the simplest machine learning models. Training it, however, uses many of the same components that other models in this book require. Therefore, before diving into the implementation details it is worth designing some of the APIs that we use throughout. Treating components in deep learning as objects, we can start by defining classes for these objects and their interactions. This object-oriented design for implementation will greatly streamline the presentation and you might even want to use it in your projects.

Inspired by open-source libraries such as PyTorch Lightning, at a high level we wish to have three classes: (i) **Module** contains models, losses, and optimization methods; (ii) **DataModule** provides data loaders for training and validation; (iii) both classes are combined using the **Trainer** class, which allows us to train models on a variety of hardware platforms. Most code in this book adapts Module and DataModule. We will touch upon the Trainer class only when we discuss GPUs, CPUs, parallel training, and optimization algorithms.

### 3.2.1. Utilities
We need a few utilities to simplify object-oriented programming in Jupyter notebooks. One of the challenges is that class definitions tend to be fairly long blocks of code. Notebook readability demands short code fragments, interspersed with explanations, a requirement incompatible with the style of programming common for Python libraries. The first utility function allows us to register functions as methods in a class after the class has been created. In fact, we can do so even after we have created instances of the class! It allows us to split the implementation of a class into multiple code blocks.

### 3.2.2. Models
The Module class is the base class of all models we will implement. At the very least we need three methods. The first, __init__, stores the learnable parameters, the training_step method accepts a data batch to return the loss value, and finally, configure_optimizers returns the optimization method, or a list of them, that is used to update the learnable parameters. Optionally we can define validation_step to report the evaluation measures. Sometimes we put the code for computing the output into a separate forward method to make it more reusable.
### 3.2.3. Data
The DataModule class is the base class for data. Quite frequently the __init__ method is used to prepare the data. This includes downloading and preprocessing if needed. The train_dataloader returns the data loader for the training dataset. A data loader is a (Python) generator that yields a data batch each time it is used. This batch is then fed into the training_step method of Module to compute the loss. There is an optional val_dataloader to return the validation dataset loader. It behaves in the same manner, except that it yields data batches for the validation_step method in Module.

### 3.2.4. Training
The Trainer class trains the learnable parameters in the Module class with data specified in DataModule. The key method is fit, which accepts two arguments: model, an instance of Module, and data, an instance of DataModule. It then iterates over the entire dataset max_epochs times to train the model. As before, we will defer the implementation of this method to later chapters.

### 3.2.5. Summary
To highlight the object-oriented design for our future deep learning implementation, the above classes simply show how their objects store data and interact with each other. We will keep enriching implementations of these classes, such as via @add_to_class, in the rest of the book. Moreover, these fully implemented classes are saved in the D2L library, a lightweight toolkit that makes structured modeling for deep learning easy. In particular, it facilitates reusing many components between projects without changing much at all. For instance, we can replace just the optimizer, just the model, just the dataset, etc.; this degree of modularity pays dividends throughout the book in terms of conciseness and simplicity (this is why we added it) and it can do the same for your own projects.


The rest part, please refer to the Chinese version.

<!-- ## 3.3. Synthetic Regression Data
### 3.3.1. Generating the Dataset
### 3.3.2. Reading the Dataset
### 3.3.3. Concise Implementation of the Data Loader
### 3.3.4. Summary

## 3.4. Linear Regression Implementation from Scratch
### 3.4.1. Defining the Model
### 3.4.2. Defining the Loss Function
### 3.4.3. Defining the Optimization Algorithm
### 3.4.4. Training
### 3.4.5. Summary

## 3.5. Concise Implementation of Linear Regression
### 3.5.1. Defining the Model
### 3.5.2. Defining the Loss Function
### 3.5.3. Defining the Optimization Algorithm
### 3.5.4. Training
### 3.5.5. Summary

## 3.6. Generalization
### 3.6.1. Training Error and Generalization Error
### 3.6.2. Underfitting or Overfitting?
### 3.6.3. Model Selection
### 3.6.4. Summary

## 3.7. Weight Decay
### 3.7.1. Norms and Weight Decay
### 3.7.2. High-Dimensional Linear Regression
### 3.7.3. Implementation from Scratch
### 3.7.4. Concise Implementation
### 3.7.5. Summary -->
