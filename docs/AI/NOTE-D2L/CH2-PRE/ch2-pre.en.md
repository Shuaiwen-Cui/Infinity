# CH02 - Preliminaries

To prepare for your dive into deep learning, you will need a few survival skills: (i) techniques for storing and manipulating data; (ii) libraries for ingesting and preprocessing data from a variety of sources; (iii) knowledge of the basic linear algebraic operations that we apply to high-dimensional data elements; (iv) just enough calculus to determine which direction to adjust each parameter in order to decrease the loss function; (v) the ability to automatically compute derivatives so that you can forget much of the calculus you just learned; (vi) some basic fluency in probability, our primary language for reasoning under uncertainty; and (vii) some aptitude for finding answers in the official documentation when you get stuck.

In short, this chapter provides a rapid introduction to the basics that you will need to follow most of the technical content in this book.

## 2.1 Data Manipulation

### 2.1.1 Data Acquisition

import `torch` and `torchvision` for data acquisition

=== "PYTORCH"

    ```python
    import torch
    ```
    
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    ```

tensor is a multi-dimensional array with support for autograd operations like `backward()`. We can specify the data type of the tensor, e.g., `float32`, `int32`, `uint8`, and `bool`.

=== "PYTORCH"

    ```python
    x = torch.arange(12)
    x
    ```

    ```text
    tensor([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11])
    ```
=== "TENSORFLOW"

    ```python
    x = tf.range(12)
    x
    ```

    ```text
    <tf.Tensor: shape=(12,), dtype=int32, numpy=array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11], dtype=int32)>
    ```
If you want to check the shape of a tensor, you can access its shape property.

=== "PYTORCH"

    ```python
    x.shape
    ```

    ```text
    torch.Size([12])
    ```
=== "TENSORFLOW"

    ```python
    x.shape
    ```

    ```text
    TensorShape([12])
    ```
If we just want to know the total number of elements in a tensor, i.e., the product of all of the shape elements, we can inspect its size. Because we are dealing with a vector here, the single element of its shape is identical to its size.

=== "PYTORCH"

    ```python
    x.numel()
    ```

    ```text
    12
    ```
=== "TENSORFLOW"

    ```python
    tf.size(x)
    ```

    ```text
    <tf.Tensor: shape=(), dtype=int32, numpy=12>
    ```

To change the shape of a tensor without altering either the number of elements or their values, we can invoke the `reshape` function. For example, we can transform our tensor, `x`, from a row vector with shape `(12,)` to a matrix with shape `(3, 4)`. This new tensor contains the exact same values as the old tensor, but it views them as a matrix organized as a stack of rows.

=== "PYTORCH"

    ```python
    x = x.reshape(3, 4)
    x
    ```

    ```text
    tensor([[ 0,  1,  2,  3],
            [ 4,  5,  6,  7],
            [ 8,  9, 10, 11]])
    ```
=== "TENSORFLOW"

    ```python
    x = tf.reshape(x, (3, 4))
    x
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=int32, numpy=
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11]], dtype=int32)>
    ```
If we only know how many columns we want but are unsure about the number of rows, we can specify this with the special value `-1`. In our case, instead of calling `x.reshape(3, 4)`, we could have equivalently called `x.reshape(-1, 4)` or `x.reshape(3, -1)`.

=== "PYTORCH"

    ```python
    x.reshape(-1, 4)
    ```

    ```text
    tensor([[ 0,  1,  2,  3],
            [ 4,  5,  6,  7],
            [ 8,  9, 10, 11]])
    ```
=== "TENSORFLOW"

    ```python
    tf.reshape(x, (-1, 4))
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=int32, numpy=
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11]], dtype=int32)>
    ```
Sometimes, we want to use all 0 or all 1 values as the initial value for a tensor. We can create a tensor full of zeros with the specified shape by calling `zeros`. Similarly, we can create tensors filled with 1 by calling `ones`.

=== "PYTORCH"

    ```python
    torch.zeros((2, 3, 4))
    ```

    ```text
    tensor([[[0., 0., 0., 0.],
             [0., 0., 0., 0.],
             [0., 0., 0., 0.]],
    
            [[0., 0., 0., 0.],
             [0., 0., 0., 0.],
             [0., 0., 0., 0.]]])
    ```
=== "TENSORFLOW"

    ```python
    tf.zeros((2, 3, 4))
    ```

    ```text
    <tf.Tensor: shape=(2, 3, 4), dtype=float32, numpy=
    array([[[0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.]],
    
           [[0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.]]], dtype=float32)>
    ```
All 1s are not as commonly used as all 0s, but we can still call `ones` for any tensor shape to get a tensor containing all 1s.

=== "PYTORCH"

    ```python
    torch.ones((2, 3, 4))
    ```

    ```text
    tensor([[[1., 1., 1., 1.],
             [1., 1., 1., 1.],
             [1., 1., 1., 1.]],
    
            [[1., 1., 1., 1.],
             [1., 1., 1., 1.],
             [1., 1., 1., 1.]]])
    ```
=== "TENSORFLOW"

    ```python
    tf.ones((2, 3, 4))
    ```

    ```text
    <tf.Tensor: shape=(2, 3, 4), dtype=float32, numpy=
    array([[[1., 1., 1., 1.],
            [1., 1., 1., 1.],
            [1., 1., 1., 1.]],
    
           [[1., 1., 1., 1.],
            [1., 1., 1., 1.],
            [1., 1., 1., 1.]]], dtype=float32)>
    ```
We can also specify the exact values for each element in the desired tensor by supplying a Python list (or list of lists) containing the numerical values.

=== "PYTORCH"

    ```python
    torch.tensor([[2, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
    ```

    ```text
    tensor([[2, 1, 4, 3],
            [1, 2, 3, 4],
            [4, 3, 2, 1]])
    ```
=== "TENSORFLOW"

    ```python
    tf.constant([[2, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=int32, numpy=
    array([[2, 1, 4, 3],
           [1, 2, 3, 4],
           [4, 3, 2, 1]], dtype=int32)>
    ```
We can create tensors whose elements are sampled randomly from a standard Gaussian (normal) distribution with a mean of 0 and a standard deviation of 1.

=== "PYTORCH"

    ```python
    torch.randn(3, 4)
    ```

    ```text
    tensor([[ 0.1835,  0.8077,  0.4194, -0.0254],
            [-0.3695, -0.1367, -0.2394, -0.0985],
            [-0.6557, -0.0235, -0.8858, -0.7724]])
    ```
=== "TENSORFLOW"

    ```python
    tf.random.normal(shape=(3, 4))
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=float32, numpy=
    array([[ 0.1835,  0.8077,  0.4194, -0.0254],
           [-0.3695, -0.1367, -0.2394, -0.0985],
           [-0.6557, -0.0235, -0.8858, -0.7724]], dtype=float32)>
    ```

### 2.1.2 Operations

Tensor supports a wide range of element-wise operations, including arithmetic operations (`+`, `-`, `*`, `/`, and `**`) and various advanced operations, such as `sin`, `cos`, and `exp`. We can call element-wise operations on any two tensors of the same shape. In the following example, we use commas to formulate a 5-element tuple, where each element is the result of an element-wise operation.

=== "PYTORCH"

    ```python
    x = torch.tensor([1.0, 2, 4, 8])
    y = torch.tensor([2, 2, 2, 2])
    x + y, x - y, x * y, x / y, x ** y
    ```

    ```text
    (tensor([ 3.,  4.,  6., 10.]),
     tensor([-1.,  0.,  2.,  6.]),
     tensor([ 2.,  4.,  8., 16.]),
     tensor([0.5000, 1.0000, 2.0000, 4.0000]),
     tensor([ 1.,  4., 16., 64.]))
    ```
=== "TENSORFLOW"

    ```python
    x = tf.constant([1.0, 2, 4, 8])
    y = tf.constant([2, 2, 2, 2])
    x + y, x - y, x * y, x / y, x ** y
    ```

    ```text
    (<tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 3.,  4.,  6., 10.], dtype=float32)>,
     <tf.Tensor: shape=(4,), dtype=float32, numpy=array([-1.,  0.,  2.,  6.], dtype=float32)>,
     <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 2.,  4.,  8., 16.], dtype=float32)>,
     <tf.Tensor: shape=(4,), dtype=float32, numpy=array([0.5, 1. , 2. , 4. ], dtype=float32)>,
     <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 1.,  4., 16., 64.], dtype=float32)>)
    ```
Many more operations can be applied element-wise, including unary operators like exponentiation.

=== "PYTORCH"

    ```python
    x.exp()
    ```

    ```text
    tensor([ 2.7183,  7.3891, 54.5981,  2.7183])
    ```
=== "TENSORFLOW"

    ```python
    tf.exp(x)
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 2.7183,  7.3891, 54.5981,  2.7183], dtype=float32)>
    ```
We can also concatenate multiple tensors together, stacking them end-to-end to form a larger tensor. We just need to provide a list of tensors and tell the system along which axis to concatenate. The example below shows what happens when we concatenate tensors along rows (axis 0, the first element of the shape) vs. columns (axis 1, the second element of the shape).

=== "PYTORCH"

    ```python
    x = torch.arange(12, dtype=torch.float32).reshape((3, 4))
    y = torch.tensor([[2.0, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
    torch.cat((x, y), dim=0), torch.cat((x, y), dim=1)
    ```

    ```text
    (tensor([[ 0.,  1.,  2.,  3.],
             [ 4.,  5.,  6.,  7.],
             [ 8.,  9., 10., 11.],
             [ 2.,  1.,  4.,  3.],
             [ 1.,  2.,  3.,  4.],
             [ 4.,  3.,  2.,  1.]]),
     tensor([[ 0.,  1.,  2.,  3.,  2.,  1.,  4.,  3.],
             [ 4.,  5.,  6.,  7.,  1.,  2.,  3.,  4.],
             [ 8.,  9., 10., 11.,  4.,  3.,  2.,  1.]]))
    ```
=== "TENSORFLOW"

    ```python
    x = tf.reshape(tf.range(12, dtype=tf.float32), (3, 4))
    y = tf.constant([[2.0, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
    tf.concat((x, y), axis=0), tf.concat((x, y), axis=1)
    ```

    ```text
    (<tf.Tensor: shape=(6, 4), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.],
           [ 4.,  5.,  6.,  7.],
           [ 8.,  9., 10., 11.],
           [ 2.,  1.,  4.,  3.],
           [ 1.,  2.,  3.,  4.],
           [ 4.,  3.,  2.,  1.]], dtype=float32)>,
     <tf.Tensor: shape=(3, 8), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.,  2.,  1.,  4.,  3.],
           [ 4.,  5.,  6.,  7.,  1.,  2.,  3.,  4.],
           [ 8.,  9., 10., 11.,  4.,  3.,  2.,  1.]], dtype=float32)>)
    ```
Sometimes, we want to construct a binary tensor via logical statements. Take `x == y` as an example. For each position, if `x` and `y` are equal at that position, the corresponding entry in the new tensor takes a value of 1, meaning that the logical statement `x == y` is true at that position; otherwise that position takes 0.

=== "PYTORCH"

    ```python
    x == y
    ```

    ```text
    tensor([[False,  True, False,  True],
            [False, False, False, False],
            [False, False, False, False]])
    ```
=== "TENSORFLOW"

    ```python
    tf.equal(x, y)
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=bool, numpy=
    array([[False,  True, False,  True],
           [False, False, False, False],
           [False, False, False, False]])>
    ```
Summing all the elements in the tensor yields a tensor with only one element.

=== "PYTORCH"

    ```python
    x.sum()
    ```

    ```text
    tensor(66.)
    ```
=== "TENSORFLOW"

    ```python
    tf.reduce_sum(x)
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=66.0>
    ```

### 2.1.3 Broadcasting Mechanism

In the above section, we saw how to perform element-wise operations on two tensors of the same shape. Under certain conditions, even when shapes differ, we can still perform element-wise operations by invoking the broadcasting mechanism. This mechanism works in the following way: First, expand one or both arrays by copying elements appropriately so that after this transformation, the two tensors have the same shape. Second, carry out the element-wise operations on the resulting arrays.

In most cases, we broadcast along an axis where an array initially only has length 1, such as in the following example.

=== "PYTORCH"

    ```python
    a = torch.arange(3).reshape((3, 1))
    b = torch.arange(2).reshape((1, 2))
    a, b
    ```

    ```text
    (tensor([[0],
             [1],
             [2]]),
     tensor([[0, 1]]))
    ```
=== "TENSORFLOW"

    ```python
    a = tf.reshape(tf.range(3), (3, 1))
    b = tf.reshape(tf.range(2), (1, 2))
    a, b
    ```

    ```text
    (<tf.Tensor: shape=(3, 1), dtype=int32, numpy=
    array([[0],
           [1],
           [2]], dtype=int32)>,
     <tf.Tensor: shape=(1, 2), dtype=int32, numpy=array([[0, 1]], dtype=int32)>)
    ```
Since `a` and `b` are `(3 × 1)` and `(1 × 2)` matrices respectively, their shapes do not match up if we want to add them. We broadcast the entries of both matrices into a larger `(3 × 2)` matrix as follows: for matrix `a` it replicates the columns and for matrix `b` it replicates the rows before adding up both element-wise.

=== "PYTORCH"

    ```python
    a + b
    ```

    ```text
    tensor([[0, 1],
            [1, 2],
            [2, 3]])
    ```
=== "TENSORFLOW"

    ```python
    a + b
    ```

    ```text
    <tf.Tensor: shape=(3, 2), dtype=int32, numpy=
    array([[0, 1],
           [1, 2],
           [2, 3]], dtype=int32)>
    ```
### 2.1.4 Indexing and Slicing

Just as in any other Python array, elements in a tensor can be accessed by index. As in any Python array, the first element has index 0 and ranges are specified to include the first but not the last element. Negative indices are commonly used to access elements relative to the end of the array. As usual, we can modify the contents of an array by specifying an index and assigning a new value.

=== "PYTORCH"

    ```python
    x[-1]
    ```

    ```text
    tensor([ 8.,  9., 10., 11.])
    ```
=== "TENSORFLOW"

    ```python
    x[-1]
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 8.,  9., 10., 11.], dtype=float32)>
    ```

Beyond reading, writing, and slicing by elements, we can also access ranges of elements. For instance, we can slice `x` to obtain the first two rows.

=== "PYTORCH"

    ```python
    x[1:3]
    ```

    ```text
    tensor([[ 4.,  5.,  6.,  7.],
            [ 8.,  9., 10., 11.]])
    ```
=== "TENSORFLOW"

    ```python
    x[1:3]
    ```

    ```text
    <tf.Tensor: shape=(2, 4), dtype=float32, numpy=
    array([[ 4.,  5.,  6.,  7.],
           [ 8.,  9., 10., 11.]], dtype=float32)>
    ```
Besides reading, writing, and slicing single elements or ranges of elements, we can also assign multiple elements with the same value. For instance, `[0, 1]` is the index for the first and second elements. Rather than assigning them one by one, we can assign them at the same time.

=== "PYTORCH"

    ```python
    x[1,2] = 9
    x
    ```

    ```text
    tensor([[ 0.,  1.,  2.,  3.],
            [ 4.,  5.,  9.,  7.],
            [ 8.,  9., 10., 11.]])
    ```
=== "TENSORFLOW"

    ```python
    x[1,2].assign(9)
    x
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.],
           [ 4.,  5.,  9.,  7.],
           [ 8.,  9., 10., 11.]], dtype=float32)>
    ```
Similarly, if we want to assign multiple elements the same value, we simply index all of them and then assign them the value. Below, we assign each element in the second and third rows to 12.

=== "PYTORCH"

    ```python
    x[1:3, :] = 12
    x
    ```

    ```text
    tensor([[ 0.,  1.,  2.,  3.],
            [12., 12., 12., 12.],
            [12., 12., 12., 12.]])
    ```
=== "TENSORFLOW"

    ```python
    x[1:3, :].assign(12)
    x
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.],
           [12., 12., 12., 12.],
           [12., 12., 12., 12.]], dtype=float32)>
    ```
### 2.1.5 Saving Memory

In some cases, we might want to perform an operation on the same array several times. If we are not careful, we will allocate new memory to store the results of each operation. For example, if we write `y = x + y`, we will dereference the tensor that `y` used to point to and instead point `y` at the newly allocated memory. In the following example, we demonstrate this with Python's `id` function, which gives us the exact address of the referenced object in memory. After running `y = y + x`, we will find that `id(y)` points to a different location. That is because Python first evaluates `y + x`, allocating new memory for the result and then subsequently redirects `y` to point at this new location in memory.

=== "PYTORCH"

    ```python
    before = id(y)
    y = y + x
    id(y) == before
    ```

    ```text
    False
    ```
=== "TENSORFLOW"

    ```python
    before = id(y)
    y = y + x
    id(y) == before
    ```

    ```text
    False
    ```

This might be undesirable for two reasons. First, we do not want to run around allocating memory unnecessarily all the time. In machine learning, we might have hundreds of megabytes of parameters and update all of them multiple times per second. Typically, we will want to perform these updates in place. Second, we might point at the same parameters from multiple variables. If we do not update in place, other references will still point to the old memory location, making it possible for parts of our code to inadvertently reference stale parameters.

Fortunately, performing in-place operations in MXNet is easy. We can assign the result of an operation to a previously allocated array with slice notation, e.g., `y[:] = <expression>`. To illustrate the behavior, we first clone the shape of another variable, allocating new memory in the process, and then perform an operation, writing the result to the newly allocated space. In this case, `id(y)` still points to the same location because we allocated no new memory for the operation.

=== "PYTORCH"

    ```python
    z = torch.zeros_like(y)
    print('id(z):', id(z))
    z[:] = x + y
    print('id(z):', id(z))
    ```

    ```text
    id(z): 140539816595456
    id(z): 140539816595456
    ```
=== "TENSORFLOW"

    ```python
    z = tf.zeros_like(y)
    print('id(z):', id(z))
    z[:] = x + y
    print('id(z):', id(z))
    ```

    ```text
    id(z): 140539816595456
    id(z): 140539816595456
    ```
### 2.1.6 Conversion to Other Python Objects

Converting to a NumPy tensor, or vice versa, is easy. The converted result does not share memory. This minor inconvenience is actually quite important: when you perform operations on the CPU or on GPUs, you do not want to halt computation, waiting to see whether the NumPy package of Python might want to be doing something else with the same chunk of memory.

=== "PYTORCH"

    ```python
    a = x.numpy()
    b = torch.tensor(a)
    type(a), type(b)
    ```

    ```text
    (numpy.ndarray, torch.Tensor)
    ```
=== "TENSORFLOW"

    ```python
    a = x.numpy()
    b = tf.convert_to_tensor(a)
    type(a), type(b)
    ```

    ```text
    (numpy.ndarray, tensorflow.python.framework.ops.EagerTensor)
    ```
To convert a size-1 tensor to a Python scalar, we can invoke the `item` function or Python's built-in functions.

=== "PYTORCH"

    ```python
    a = torch.tensor([3.5])
    a, a.item(), float(a), int(a)
    ```

    ```text
    (tensor([3.5000]), 3.5, 3.5, 3)
    ```
=== "TENSORFLOW"

    ```python
    a = tf.constant([3.5])
    a, a.numpy(), float(a), int(a)
    ```

    ```text
    (<tf.Tensor: shape=(1,), dtype=float32, numpy=array([3.5], dtype=float32)>,
     array([3.5], dtype=float32),
     3.5,
     3)
    ```

### 2.1.7 Summary

The tensor class is the main interface for storing and manipulating data in deep learning libraries. Tensors provide a variety of functionalities including construction routines; indexing and slicing; basic mathematics operations; broadcasting; memory-efficient assignment; and conversion to and from other Python objects.






## 2.2 Data Preprocessing

## 2.3 Linear Algebra

## 2.4 Calculus

## 2.5 Automatic Differentiation

## 2.6 Probability and Statistics

## 2.7 Documentation