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

So far, we have been working with synthetic data that arrived in ready-made tensors. However, to apply deep learning in the wild we must extract messy data stored in arbitrary formats, and preprocess it to suit our needs. Fortunately, the pandas library can do much of the heavy lifting. This section, while no substitute for a proper pandas tutorial, will give you a crash course on some of the most common routines.\

### 2.2.1 Reading the Dataset

Comma-separated values (CSV) files are ubiquitous for the storing of tabular (spreadsheet-like) data. In them, each line corresponds to one record and consists of several (comma-separated) fields, e.g., “Albert Einstein,March 14 1879,Ulm,Federal polytechnic school,field of gravitational physics”. To demonstrate how to load CSV files with pandas, we create a CSV file below ../data/house_tiny.csv. This file represents a dataset of homes, where each row corresponds to a distinct home and the columns correspond to the number of rooms (NumRooms), the roof type (RoofType), and the price (Price).

=== "PYTORCH"

    ```python
    import os

    os.makedirs(os.path.join('..', 'data'), exist_ok=True)
    data_file = os.path.join('..', 'data', 'house_tiny.csv')
    with open(data_file, 'w') as f:
        f.write('NumRooms,Alley,Price\n')  # Column names
        f.write('NA,Pave,127500\n')  # Each row represents a data example
        f.write('2,NA,106000\n')
        f.write('4,NA,178100\n')
        f.write('NA,NA,140000\n')
    ```
Now let's import pandas and load the dataset with `read_csv`. The argument `na_values` means “not available values”.

=== "PYTORCH"

    ```python
    import pandas as pd
    data = pd.read_csv(data_file)
    print(data)
    ```

    ```text
       NumRooms Alley   Price
    0       NaN  Pave  127500
    1       2.0   NaN  106000
    2       4.0   NaN  178100
    3       NaN   NaN  140000
    ```
=== "TENSORFLOW"

    ```python
    import pandas as pd
    data = pd.read_csv(data_file)
    print(data)
    ```

    ```text
       NumRooms Alley   Price
    0       NaN  Pave  127500
    1       2.0   NaN  106000
    2       4.0   NaN  178100
    3       NaN   NaN  140000
    ```
### 2.2.2 Data Preparation

In supervised learning, we train models to predict a designated target value, given some set of input values. Our first step in processing the dataset is to separate out columns corresponding to input versus target values. We can select columns either by name or via integer-location based indexing (iloc).

You might have noticed that pandas replaced all CSV entries with value NA with a special NaN (not a number) value. This can also happen whenever an entry is empty, e.g., “3,,,270000”. These are called missing values and they are the “bed bugs” of data science, a persistent menace that you will confront throughout your career. Depending upon the context, missing values might be handled either via imputation or deletion. Imputation replaces missing values with estimates of their values while deletion simply discards either those rows or those columns that contain missing values.

Here are some common imputation heuristics. For categorical input fields, we can treat NaN as a category. Since the RoofType column takes values Slate and NaN, pandas can convert this column into two columns RoofType_Slate and RoofType_nan. A row whose roof type is Slate will set values of RoofType_Slate and RoofType_nan to 1 and 0, respectively. The converse holds for a row with a missing RoofType value.

=== "PYTORCH"

    ```python
    inputs, targets = data.iloc[:, 0:2], data.iloc[:, 2]
    inputs = pd.get_dummies(inputs, dummy_na=True)
    print(inputs)
    ```

    ```text
       NumRooms  RoofType_Slate  RoofType_nan
    0       NaN           False          True
    1       2.0           False          True
    2       4.0            True         False
    3       NaN           False          True
    ```
=== "TENSORFLOW"

    ```python
    inputs, targets = data.iloc[:, 0:2], data.iloc[:, 2]
    inputs = pd.get_dummies(inputs, dummy_na=True)
    print(inputs)
    ```

    ```text
       NumRooms  RoofType_Slate  RoofType_nan
    0       NaN           False          True
    1       2.0           False          True
    2       4.0            True         False
    3       NaN           False          True
    ```
For missing numerical values, one common heuristic is to replace the NaN entries with the mean value of the corresponding column.

=== "PYTORCH"

    ```python
    # Fill missing values with the column mean
    inputs = inputs.fillna(inputs.mean())
    print(inputs)
    ```

    ```text
       NumRooms  RoofType_Slate  RoofType_nan
    0       3.0           False          True
    1       2.0           False          True
    2       4.0            True         False
    3       3.0           False          True
    ```
=== "TENSORFLOW"

    ```python
    # Fill missing values with the column mean
    inputs = inputs.fillna(inputs.mean())
    print(inputs)
    ```

    ```text
       NumRooms  RoofType_Slate  RoofType_nan
    0       3.0           False          True
    1       2.0           False          True
    2       4.0            True         False
    3       3.0           False          True
    ```
### 2.2.3 Conversion to the Tensor Format

Now that all the entries in inputs and targets are numerical, we can load them into a tensor (recall Section 2.1).

=== "PYTORCH"

    ```python
    import torch
    X, y = torch.tensor(inputs.values), torch.tensor(targets.values)
    X, y
    ```

    ```text
    (tensor([[3., 0., 1.],
             [2., 0., 1.],
             [4., 1., 0.],
             [3., 0., 1.]], dtype=torch.float64),
     tensor([127500, 106000, 178100, 140000]))
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    X, y = tf.constant(inputs.values), tf.constant(targets.values)
    X, y
    ```

    ```text
    (<tf.Tensor: shape=(4, 3), dtype=float64, numpy=
    array([[3., 0., 1.],
           [2., 0., 1.],
           [4., 1., 0.],
           [3., 0., 1.]])>,
     <tf.Tensor: shape=(4,), dtype=int64, numpy=array([127500, 106000, 178100, 140000])>)
    ```
### 2.2.4 Summary
You now know how to partition data columns, impute missing variables, and load pandas data into tensors. In Section 5.7, you will pick up some more data processing skills. While this crash course kept things simple, data processing can get hairy. For example, rather than arriving in a single CSV file, our dataset might be spread across multiple files extracted from a relational database. For instance, in an e-commerce application, customer addresses might live in one table and purchase data in another. Moreover, practitioners face myriad data types beyond categorical and numeric, for example, text strings, images, audio data, and point clouds. Oftentimes, advanced tools and efficient algorithms are required in order to prevent data processing from becoming the biggest bottleneck in the machine learning pipeline. These problems will arise when we get to computer vision and natural language processing. Finally, we must pay attention to data quality. Real-world datasets are often plagued by outliers, faulty measurements from sensors, and recording errors, which must be addressed before feeding the data into any model. Data visualization tools such as seaborn, Bokeh, or matplotlib can help you to manually inspect the data and develop intuitions about the type of problems you may need to address.

## 2.3 Linear Algebra

### 2.3.1 Scalars

If you never studied linear algebra or machine learning, then you might be surprised to see so many boldfaced letters. These are not typos! We use boldfaced lowercase letters (e.g., $\mathbf{x}$) to refer to vectors and boldfaced uppercase letters (e.g., $\mathbf{X}$) to denote matrices. For scalars (single numbers), we use regular letters (e.g., $x$). We also work with tensors, which are arrays with more than two axes. We use calligraphic font (e.g., $\mathcal{X}$) for tensors.

We use $\mathbb{R}$ to denote the set containing all scalar real-valued numbers. By convention, we use calligraphic font (e.g., $\mathcal{X}$) for sets. This is because we can think of a scalar as a set containing just one element. By extension, $\mathbb{R}^n$ denotes the set of all vectors consisting of $n$ real-valued scalars. In machine learning, we typically work with sets of scalars, vectors, matrices, or tensors of various sizes and shapes, so it is important to keep the distinctions among them straight.

Scalars are implemented as tensors that contain only one element. Below, we assign two scalars and perform the familiar addition, multiplication, division, and exponentiation operations.

=== "PYTORCH"

    ```python
    import torch
    x = torch.tensor(3.0)
    y = torch.tensor(2.0)
    x + y, x * y, x / y, x ** y
    ```

    ```text
    (tensor(5.), tensor(6.), tensor(1.5000), tensor(9.))
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    x = tf.constant(3.0)
    y = tf.constant(2.0)
    x + y, x * y, x / y, x ** y
    ```

    ```text
    (<tf.Tensor: shape=(), dtype=float32, numpy=5.0>,
     <tf.Tensor: shape=(), dtype=float32, numpy=6.0>,
     <tf.Tensor: shape=(), dtype=float32, numpy=1.5>,
     <tf.Tensor: shape=(), dtype=float32, numpy=9.0>)
    ```
### 2.3.2 Vectors

You can think of a vector as simply a list of scalar values. We call these values the *elements* (or *entries*) of the vector. When our vectors represent examples from our dataset, its elements correspond to features of our examples. For example, if we collected data on the height, weight, and age of a group of people, we could represent a single person as a vector $\mathbf{x} = [x_1, x_2, x_3]$ with elements corresponding to height, weight, and age, respectively. Throughout this book, we work with column vectors, which are represented as *n*-dimensional arrays with *n* rows and 1 column. Each element $x_i$ of a vector $\mathbf{x}$ is indicated by its index $i$.

In math notation, we will usually denote vectors as bold-faced, upright letters (e.g., $\mathbf{x}$). In the source code of the book, we denote them as variables with arrow on top (e.g., `x`). In either case, we use lowercase letters with either no decoration (e.g., $x$) or arrows (e.g., $\vec{x}$) to refer to their elements.

We can create a row vector `x` containing the first $n$ natural numbers with `arange`. Then we access any element by indexing into the tensor.

=== "PYTORCH"

    ```python
    x = torch.arange(4)
    x
    ```

    ```text
    tensor([0, 1, 2, 3])
    ```
=== "TENSORFLOW"

    ```python
    x = tf.range(4)
    x
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=int32, numpy=array([0, 1, 2, 3], dtype=int32)>
    ```

We can refer to any element of a vector by using a subscript. For example, we can refer to the $i^\mathrm{th}$ element of $\mathbf{x}$ by $x_i$. Note that the element $x_i$ is a scalar, so we do not boldface the font when referring to it. Also, note that the element $x_i$ is a scalar, so we do not boldface the font when referring to it.

=== "PYTORCH"

    ```python
    x[3]
    ```

    ```text
    tensor(3)
    ```
=== "TENSORFLOW"

    ```python
    x[3]
    ```

    ```text
    <tf.Tensor: shape=(), dtype=int32, numpy=3>
    ```
You can check the length of the vector with `len(x)`.

=== "PYTORCH"

    ```python
    len(x)
    ```

    ```text
    4
    ```
=== "TENSORFLOW"

    ```python
    len(x)
    ```

    ```text
    4
    ```

You can check the shape of the vector with `x.shape`.

=== "PYTORCH"

    ```python
    x.shape
    ```

    ```text
    torch.Size([4])
    ```
=== "TENSORFLOW"

    ```python
    x.shape
    ```

    ```text
    TensorShape([4])
    ```
### 2.3.3 Matrices

Just as vectors generalize scalars from order 0 to order 1, matrices generalize vectors from order 1 to order 2. Matrices, which we will typically denote with bold-faced, capital letters (e.g., $\mathbf{X}$), are represented in code as arrays with two axes. Visually, we can illustrate any matrix $\mathbf{X} \in \mathbb{R}^{m \times n}$ with a table, where each element $x_{ij}$ belongs to the $i^{\mathrm{th}}$ row and $j^{\mathrm{th}}$ column:

$$\mathbf{X} = \begin{bmatrix} x_{11} & x_{12} & \cdots & x_{1n} \\ x_{21} & x_{22} & \cdots & x_{2n} \\ \vdots & \vdots & \ddots & \vdots \\ x_{m1} & x_{m2} & \cdots & x_{mn} \end{bmatrix}.$$

We can create a matrix `A` with $m$ rows and $n$ columns with `arange` and `reshape`.

=== "PYTORCH"

    ```python
    A = torch.arange(20).reshape(5, 4)
    A
    ```

    ```text
    tensor([[ 0,  1,  2,  3],
            [ 4,  5,  6,  7],
            [ 8,  9, 10, 11],
            [12, 13, 14, 15],
            [16, 17, 18, 19]])
    ```
=== "TENSORFLOW"

    ```python
    A = tf.range(20).reshape(5, 4)
    A
    ```

    ```text
    <tf.Tensor: shape=(5, 4), dtype=int32, numpy=
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11],
           [12, 13, 14, 15],
           [16, 17, 18, 19]], dtype=int32)>
    ```

We can access the scalar element $x_{ij}$ of a matrix $\mathbf{X}$ by specifying the indices for the row ($i$) and column ($j$), such as `A[i, j]`.

=== "PYTORCH"

    ```python
    A[2, 3]
    ```

    ```text
    tensor(11)
    ```
=== "TENSORFLOW"

    ```python
    A[2, 3]
    ```

    ```text
    <tf.Tensor: shape=(), dtype=int32, numpy=11>
    ```

We can transpose the matrix through `A.T`.

=== "PYTORCH"

    ```python
    A.T
    ```

    ```text
    tensor([[ 0,  4,  8, 12, 16],
            [ 1,  5,  9, 13, 17],
            [ 2,  6, 10, 14, 18],
            [ 3,  7, 11, 15, 19]])
    ```
=== "TENSORFLOW"

    ```python
    tf.transpose(A)
    ```

    ```text
    <tf.Tensor: shape=(4, 5), dtype=int32, numpy=
    array([[ 0,  4,  8, 12, 16],
           [ 1,  5,  9, 13, 17],
           [ 2,  6, 10, 14, 18],
           [ 3,  7, 11, 15, 19]], dtype=int32)>
    ```
### 2.3.4 Tensors

Just as vectors generalize scalars, and matrices generalize vectors, we can build data structures with even more axes. Tensors give us a generic way of describing $n$-dimensional arrays with an arbitrary number of axes. Vectors, for example, are first-order tensors, and matrices are second-order tensors. Tensors will become more important when we start working with images, which arrive as 3D data structures. For now, we will skip the fancy notation and just think of tensors as multidimensional arrays. We can create a tensor `X` with 3 axes and sizes $2 \times 3 \times 4$.

=== "PYTORCH"

    ```python
    X = torch.arange(24).reshape(2, 3, 4)
    X
    ```

    ```text
    tensor([[[ 0,  1,  2,  3],
             [ 4,  5,  6,  7],
             [ 8,  9, 10, 11]],

            [[12, 13, 14, 15],
             [16, 17, 18, 19],
             [20, 21, 22, 23]]])
    ```
=== "TENSORFLOW"

    ```python
    X = tf.range(24).reshape(2, 3, 4)
    X
    ```

    ```text
    <tf.Tensor: shape=(2, 3, 4), dtype=int32, numpy=
    array([[[ 0,  1,  2,  3],
            [ 4,  5,  6,  7],
            [ 8,  9, 10, 11]],

           [[12, 13, 14, 15],
            [16, 17, 18, 19],
            [20, 21, 22, 23]]], dtype=int32)>
    ```

Tensors can be even more useful than matrices. For instance, we can stack matrices to obtain a 3D tensor with a shape of $3 \times 3 \times 4$.

=== "PYTORCH"

    ```python
    A = torch.arange(20, dtype=torch.float32).reshape(5, 4)
    B = A.clone()  # Assign a copy of A to B by allocating new memory
    A, A + B
    ```

    ```text
    (tensor([[ 0.,  1.,  2.,  3.],
             [ 4.,  5.,  6.,  7.],
             [ 8.,  9., 10., 11.],
             [12., 13., 14., 15.],
             [16., 17., 18., 19.]]),
     tensor([[ 0.,  2.,  4.,  6.],
             [ 8., 10., 12., 14.],
             [16., 18., 20., 22.],
             [24., 26., 28., 30.],
             [32., 34., 36., 38.]]))
    ```
=== "TENSORFLOW"

    ```python
    A = tf.range(20, dtype=tf.float32).reshape(5, 4)
    B = tf.Variable(A)  # Assign a copy of A to B by allocating new memory
    A, A + B
    ```

    ```text
    (<tf.Tensor: shape=(5, 4), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.],
           [ 4.,  5.,  6.,  7.],
           [ 8.,  9., 10., 11.],
           [12., 13., 14., 15.],
           [16., 17., 18., 19.]], dtype=float32)>,
     <tf.Tensor: shape=(5, 4), dtype=float32, numpy=
    array([[ 0.,  2.,  4.,  6.],
           [ 8., 10., 12., 14.],
           [16., 18., 20., 22.],
           [24., 26., 28., 30.],
           [32., 34., 36., 38.]], dtype=float32)>)
    ```
### 2.3.5 Basic Properties of Tensor Arithmetic

Scalars, vectors, matrices, and higher-order tensors all have some handy properties. For example, elementwise operations produce outputs that have the same shape as their operands.

=== "PYTORCH"

    ```python
    A = torch.arange(6, dtype=torch.float32).reshape(2, 3)
    B = A.clone()  # Assign a copy of A to B by allocating new memory
    A, A + B
    ```

    ```text
    (tensor([[0., 1., 2.],
             [3., 4., 5.]]),
     tensor([[ 0.,  2.,  4.],
             [ 6.,  8., 10.]]))
    ```
=== "TENSORFLOW"

    ```python
    A = tf.reshape(tf.range(6, dtype=tf.float32), (2, 3))
    B = A  # No cloning of A to B by allocating new memory
    A, A + B
    ```

    ```text
    (<tf.Tensor: shape=(2, 3), dtype=float32, numpy=
    array([[0., 1., 2.],
           [3., 4., 5.]], dtype=float32)>,
     <tf.Tensor: shape=(2, 3), dtype=float32, numpy=
    array([[ 0.,  2.,  4.],
           [ 6.,  8., 10.]], dtype=float32)>)
    ```

The elementwise product of two matrices is called their Hadamard product (denoted $\odot$). We can spell out the entries of the Hadamard product of two matrices $\mathbf{A}, \mathbf{B} \in \mathbb{R}^{m \times n}$:

$$(\mathbf{A} \odot \mathbf{B})_{ij} = \begin{bmatrix} a_{11} b_{11} & a_{12} b_{12} & \cdots & a_{1n} b_{1n} \\ a_{21} b_{21} & a_{22} b_{22} & \cdots & a_{2n} b_{2n} \\ \vdots & \vdots & \ddots & \vdots \\ a_{m1} b_{m1} & a_{m2} b_{m2} & \cdots & a_{mn} b_{mn} \end{bmatrix}.$$

=== "PYTORCH"

    ```python
    A * B
    ```

    ```text
    tensor([[ 0.,  1.,  4.],
            [ 9., 16., 25.]])
    ```
=== "TENSORFLOW"

    ```python
    A * B
    ```

    ```text
    <tf.Tensor: shape=(2, 3), dtype=float32, numpy=
    array([[ 0.,  1.,  4.],
           [ 9., 16., 25.]], dtype=float32)>
    ```
### 2.3.6 Reduction

Often, we wish to calculate the sum of a tensor’s elements. To express the sum of the elements in a vector $x$ of length n, we use $\sum_{i=1}^n x_i$. In code, we can call the method `sum`.

=== "PYTORCH"

    ```python
    x = torch.arange(3, dtype=torch.float32)
    x, x.sum()
    ```

    ```text
    (tensor([0., 1., 2.]), tensor(3.))
    ```
=== "TENSORFLOW"

    ```python
    x = tf.range(3, dtype=tf.float32)
    x, tf.reduce_sum(x)
    ```

    ```text
    (<tf.Tensor: shape=(3,), dtype=float32, numpy=array([0., 1., 2.], dtype=float32)>,
     <tf.Tensor: shape=(), dtype=float32, numpy=3.0>)
    ```

To express sums over the elements of tensors of arbitrary shape, we simply sum over all its axes.

=== "PYTORCH"

    ```python
    A.shape, A.sum()
    ```

    ```text
    (torch.Size([5, 4]), tensor(190.))
    ```
=== "TENSORFLOW"

    ```python
    A.shape, tf.reduce_sum(A)
    ```

    ```text
    (TensorShape([5, 4]), <tf.Tensor: shape=(), dtype=float32, numpy=190.0>)
    ```
Specifying axis=1 will reduce the column dimension (axis 1) by summing up elements of all the columns.

=== "PYTORCH"

    ```python
    A_sum_axis1 = A.sum(axis=1)
    A_sum_axis1, A_sum_axis1.shape
    ```

    ```text
    (tensor([ 6., 22., 38., 54., 70.]), torch.Size([5]))
    ```
=== "TENSORFLOW"

    ```python
    A_sum_axis1 = tf.reduce_sum(A, axis=1)
    A_sum_axis1, A_sum_axis1.shape
    ```

    ```text
    (<tf.Tensor: shape=(5,), dtype=float32, numpy=array([ 6., 22., 38., 54., 70.], dtype=float32)>,
     TensorShape([5]))
    ```
Reducing a matrix along both rows and columns via summation is equivalent to summing up all the elements of the matrix.

=== "PYTORCH"

    ```python
    A.sum(axis=[0, 1])
    ```

    ```text
    tensor(190.)
    ```

=== "TENSORFLOW"

    ```python
    tf.reduce_sum(A, axis=[0, 1])
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=190.0>
    ```

A related quantity is the mean, also called the average. We calculate the mean by dividing the sum by the total number of elements. Because computing the mean is so common, it gets a dedicated library function that works analogously to sum.

=== "PYTORCH"

    ```python
    A.mean(), A.sum() / A.numel()
    ```

    ```text
    (tensor(9.5000), tensor(9.5000))
    ```
=== "TENSORFLOW"

    ```python
    tf.reduce_mean(A), tf.reduce_sum(A) / tf.size(A).numpy()
    ```

    ```text
    (<tf.Tensor: shape=(), dtype=float32, numpy=9.5>,
     <tf.Tensor: shape=(), dtype=float32, numpy=9.5>)
    ```
Likewise, the function for calculating the mean can also reduce a tensor along specific axes.

=== "PYTORCH"

    ```python
    A.mean(axis=0), A.sum(axis=0) / A.shape[0]
    ```

    ```text
    (tensor([ 8.,  9., 10., 11.]), tensor([ 8.,  9., 10., 11.]))
    ```
=== "TENSORFLOW"

    ```python
    tf.reduce_mean(A, axis=0), tf.reduce_sum(A, axis=0) / A.shape[0]
    ```

    ```text
    (<tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 8.,  9., 10., 11.], dtype=float32)>,
     <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 8.,  9., 10., 11.], dtype=float32)>)
    ```

### 2.3.7 Non-Reduction Sum

Sometimes it can be useful to keep the number of axes unchanged when invoking the function for calculating the sum or mean. This matters when we want to use the broadcast mechanism.

=== "PYTORCH"

    ```python
    sum_A = A.sum(axis=1, keepdims=True)
    sum_A
    ```

    ```text
    tensor([[ 6.],
            [22.],
            [38.],
            [54.],
            [70.]])
    ```
=== "TENSORFLOW"

    ```python
    sum_A = tf.reduce_sum(A, axis=1, keepdims=True)
    sum_A
    ```

    ```text
    <tf.Tensor: shape=(5, 1), dtype=int32, numpy=
    array([[ 6],
           [22],
           [38],
           [54],
           [70]], dtype=int32)>
    ```
For instance, since sum_A keeps its two axes after summing each row, we can divide A by sum_A with broadcasting to create a matrix where each row sums up to 1.

=== "PYTORCH"

    ```python
    A / sum_A
    ```

    ```text
    tensor([[0.0000, 0.1667, 0.3333, 0.5000],
            [0.1818, 0.2273, 0.2727, 0.3182],
            [0.2105, 0.2368, 0.2632, 0.2895],
            [0.2222, 0.2407, 0.2593, 0.2778],
            [0.2286, 0.2429, 0.2571, 0.2714]])
    ```
=== "TENSORFLOW"

    ```python
    A / sum_A
    ```

    ```text
    <tf.Tensor: shape=(5, 4), dtype=float32, numpy=
    array([[0.        , 0.16666667, 0.33333334, 0.5       ],
           [0.18181819, 0.22727273, 0.27272728, 0.3181818 ],
           [0.21052632, 0.23684211, 0.2631579 , 0.28947368],
           [0.22222222, 0.24074075, 0.25925925, 0.2777778 ],
           [0.22857143, 0.24285714, 0.25714287, 0.2714286 ]],
          dtype=float32)>
    ```
If we want to calculate the cumulative sum of elements of A along some axis, say axis=0 (row by row), we can call the cumsum function. By design, this function does not reduce the input tensor along any axis.

=== "PYTORCH"

    ```python
    A.cumsum(axis=0)
    ```

    ```text
    tensor([[ 0.,  1.,  2.,  3.],
            [ 4.,  6.,  8., 10.],
            [12., 15., 18., 21.],
            [24., 28., 32., 36.],
            [40., 45., 50., 55.]])
    ```
=== "TENSORFLOW"

    ```python
    tf.cumsum(A, axis=0)
    ```

    ```text
    <tf.Tensor: shape=(5, 4), dtype=int32, numpy=
    array([[ 0,  1,  2,  3],
           [ 4,  6,  8, 10],
           [12, 15, 18, 21],
           [24, 28, 32, 36],
           [40, 45, 50, 55]], dtype=int32)>
    ```
### 2.3.8 Dot Products

So far, we have only performed elementwise operations, sums, and averages. And if this was all we could do, linear algebra would not deserve its own section. Fortunately, this is where things get more interesting. One of the most fundamental operations is the dot product. Given two vectors $\mathbf{x}, \mathbf{y} \in \mathbb{R}^d$, their dot product $\mathbf{x}^\top \mathbf{y}$ (or $\langle \mathbf{x}, \mathbf{y} \rangle$) is a sum over the products of the elements at the same position: $\mathbf{x}^\top \mathbf{y} = \sum_{i=1}^d x_i y_i$.

=== "PYTORCH"

    ```python
    y = torch.ones(4, dtype=torch.float32)
    x, y, torch.dot(x, y)
    ```

    ```text
    (tensor([0., 1., 2., 3.]), tensor([1., 1., 1., 1.]), tensor(6.))
    ```
=== "TENSORFLOW"

    ```python
    y = tf.ones(4, dtype=tf.float32)
    x, y, tf.tensordot(x, y, axes=1)
    ```

    ```text
    (<tf.Tensor: shape=(4,), dtype=float32, numpy=array([0., 1., 2., 3.], dtype=float32)>,
     <tf.Tensor: shape=(4,), dtype=float32, numpy=array([1., 1., 1., 1.], dtype=float32)>,
     <tf.Tensor: shape=(), dtype=float32, numpy=6.0>)
    ```

Equivalently, we can calculate the dot product of two vectors by performing an elementwise multiplication followed by a sum:

$$\mathbf{x}^\top \mathbf{y} = \sum_{i=1}^d x_i y_i.$$

=== "PYTORCH"

    ```python
    torch.sum(x * y)
    ```

    ```text
    tensor(6.)
    ```
=== "TENSORFLOW"

    ```python
    tf.reduce_sum(x * y)
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=6.0>
    ```

Dot products are useful in a wide range of contexts. For example, given some set of values, denoted by a vector $\mathbf{x} \in \mathbb{R}^d$ and a set of weights denoted by $\mathbf{w} \in \mathbb{R}^d$, the weighted sum of the values in $\mathbf{x}$ according to the weights $\mathbf{w}$ could be expressed as the dot product $\mathbf{x}^\top \mathbf{w}$. When the weights are non-negative and sum to one (i.e., $\left(\sum_{i=1}^d {w_i} = 1\right)$), the dot product expresses a *weighted average*. We revisit this example in the section on softmax operations (see Section 2.5).

### 2.3.9 Matrix-Vector Products

Now that we know how to calculate dot products, we can begin to understand the product between an $m \times n$ matrix $\mathbf{A}$ and an $n$-dimensional vector $x$. To start off, we visualize our matrix in terms of its row vectors:

$$\mathbf{A} = \begin{bmatrix} \mathbf{a}^\top_{1} \\ \mathbf{a}^\top_{2} \\ \vdots \\ \mathbf{a}^\top_{m} \end{bmatrix},$$

where each $\mathbf{a}^\top_{i} \in \mathbb{R}^n$ is a row vector representing the $i^\mathrm{th}$ row of the matrix $\mathbf{A}$. 

The maxtri-vector product $\mathbf{A}\mathbf{x}$ is simply a column vector of length $m$ whose $i^\mathrm{th}$ element is the dot product $\mathbf{a}^\top_i \mathbf{x}$:

$$\mathbf{A} = \begin{bmatrix} \mathbf{a}^\top_{1} \\ \mathbf{a}^\top_{2} \\ \vdots \\ \mathbf{a}^\top_{m} \end{bmatrix} \quad \mathbf{A}\mathbf{x} = \begin{bmatrix} \mathbf{a}^\top_{1} \mathbf{x} \\ \mathbf{a}^\top_{2} \mathbf{x} \\ \vdots \\ \mathbf{a}^\top_{m} \mathbf{x} \end{bmatrix}.$$

We can think of the matrix-vector product $\mathbf{A}\mathbf{x}$ as simply performing $m$ matrix-vector products and stitching the results together to form a vector of length $m$.

=== "PYTORCH"

    ```python
    A.shape, x.shape, torch.mv(A, x)
    ```

    ```text
    (torch.Size([5, 4]), torch.Size([4]), tensor([14., 38., 62., 86., 110.]))
    ```
=== "TENSORFLOW"

    ```python
    A.shape, x.shape, tf.linalg.matvec(A, x)
    ```

    ```text
    (TensorShape([5, 4]),
     TensorShape([4]),
     <tf.Tensor: shape=(5,), dtype=int32, numpy=array([14, 38, 62, 86, 110], dtype=int32)>)
    ```
### 2.3.10 Matrix-Matrix Multiplication
Once you have gotten the hang of dot products and matrix–vector products, then matrix–matrix multiplication should be straightforward.

Say that we have two matrices $\mathbf{A} \in \mathbb{R}^{n \times k}$ and $\mathbf{B} \in \mathbb{R}^{k \times m}$:

$$\mathbf{A} = \begin{bmatrix} \mathbf{a_{11}} & \mathbf{a_{12}} & \cdots & \mathbf{a_{1k}} \\ \mathbf{a_{21}} & \mathbf{a_{22}} & \cdots & \mathbf{a_{2k}} \\ \vdots & \vdots & \ddots & \vdots \\ \mathbf{a_{n1}} & \mathbf{a_{n2}} & \cdots & \mathbf{a_{nk}} \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} \mathbf{b_{11}} & \mathbf{b_{12}} & \cdots & \mathbf{b_{1m}} \\ \mathbf{b_{21}} & \mathbf{b_{22}} & \cdots & \mathbf{b_{2m}} \\ \vdots & \vdots & \ddots & \vdots \\ \mathbf{b_{k1}} & \mathbf{b_{k2}} & \cdots & \mathbf{b_{km}} \end{bmatrix}.$$

Let us focus on computing the matrix product $\mathbf{C} = \mathbf{A}\mathbf{B}$, where $\mathbf{C} \in \mathbb{R}^{n \times m}$. To produce the matrix product $\mathbf{C}$, we will carry out $n$ matrix-vector products and form the matrix $\mathbf{C}$ by concatenating the results together into a matrix with $n$ rows.

$$\mathbf{C} = \begin{bmatrix} \mathbf{a_{11}} \mathbf{B} & \mathbf{a_{12}} \mathbf{B} & \cdots & \mathbf{a_{1k}} \mathbf{B} \\ \mathbf{a_{21}} \mathbf{B} & \mathbf{a_{22}} \mathbf{B} & \cdots & \mathbf{a_{2k}} \mathbf{B} \\ \vdots & \vdots & \ddots & \vdots \\ \mathbf{a_{n1}} \mathbf{B} & \mathbf{a_{n2}} \mathbf{B} & \cdots & \mathbf{a_{nk}} \mathbf{B} \end{bmatrix}.$$

Note that $\mathbf{a_{ij}} \mathbf{B}$ is a matrix-vector product. Here, we are writing $\mathbf{a_{ij}}$ to denote the row vector $\mathbf{A}[i,:]$ and $\mathbf{a_{ij}} \mathbf{B}$ still denotes a matrix-matrix product but one in which we multiply a row vector by a matrix.

=== "PYTORCH"

    ```python
    B = torch.ones(3, 4)
    torch.mm(A, B), A@B
    ```

    ```text
    (tensor([[ 3.,  3.,  3.,  3.],
         [12., 12., 12., 12.]]),
    tensor([[ 3.,  3.,  3.,  3.],
         [12., 12., 12., 12.]]))
    ```
=== "TENSORFLOW"

    ```python
    B = tf.ones((3, 4))
    tf.matmul(A, B), A@B
    ```

    ```text
    (<tf.Tensor: shape=(2, 4), dtype=int32, numpy=
    array([[ 3,  3,  3,  3],
           [12, 12, 12, 12]], dtype=int32)>,
     <tf.Tensor: shape=(2, 4), dtype=int32, numpy=
    array([[ 3,  3,  3,  3],
           [12, 12, 12, 12]], dtype=int32)>)
    ```
### 2.3.11 Norms

Some of the most useful operators in linear algebra are norms. Informally, the norm of a vector tells us how big it is. For instance, the $l_2$  norm measures the (Euclidean) length of a vector. Here, we are employing a notion of size that concerns the magnitude of a vector’s components (not its dimensionality).

A norm is a function $f$ that maps a vector to a scalar, satisfying a handful of properties. Given any vector $\mathbf{x}$, the first property says that if we scale all the elements of a vector by a constant factor $\alpha$, its norm also scales by the *absolute value* of the same constant factor:

$$f(\alpha \mathbf{x}) = |\alpha| f(\mathbf{x}).$$

The second property is the familiar triangle inequality:

$$f(\mathbf{x} + \mathbf{y}) \leq f(\mathbf{x}) + f(\mathbf{y}).$$

The third property simply says that the norm must be non-negative:

$$f(\mathbf{x}) \geq 0.$$

That makes sense, as in most contexts the smallest *size* for anything is 0. The final property requires that the smallest norm is achieved and only achieved by a vector consisting of all zeros.

$$\forall i, [\mathbf{x}]_i = 0 \Leftrightarrow f(\mathbf{x})=0.$$

You might notice that norms sound a lot like measures of distance. And with good reason! In fact, *the $l_2$ norm of a vector is the square root of the total squared distance of the vector*:

$$\|\mathbf{x}\|_2 = \sqrt{\sum_{i=1}^n x_i^2},$$

where the subscript $2$ is often omitted in $l_2$ norms. In the above equation, the notation $\mathbf{x}_i$ refers to the scalar element of a vector $\mathbf{x}$.

In code, we can calculate the $l_2$ norm of a vector by calling `norm`.

=== "PYTORCH"

    ```python
    u = torch.tensor([3.0, -4.0])
    torch.norm(u)
    ```

    ```text
    tensor(5.)
    ```
=== "TENSORFLOW"

    ```python
    u = tf.constant([3.0, -4.0])
    tf.norm(u)
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=5.0>
    ```

In deep learning, we work more often with the squared $l_2$ norm. You might have noticed this from the formula for the objective function of linear regression, which involves a sum of squared terms. We can calculate this quantity via `torch.dot`.

=== "PYTORCH"

    ```python
    torch.dot(u, u)
    ```

    ```text
    tensor(25.)
    ```
=== "TENSORFLOW"

    ```python
    tf.tensordot(u, u, axes=1)
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=25.0>
    ```

The $l_1$ norm of a vector is calculated as the sum of the absolute values of the vector’s components:

$$\|\mathbf{x}\|_1 = \sum_{i=1}^n \left|x_i \right|.$$

As compared with the $l_2$ norm, it is less influenced by outliers. To calculate the $l_1$ norm, we compose `abs` and `sum`.

=== "PYTORCH"

    ```python
    torch.abs(u).sum()
    ```

    ```text
    tensor(7.)
    ```
=== "TENSORFLOW"

    ```python
    tf.reduce_sum(tf.abs(u))
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=7.0>
    ```

## 2.4 Calculus

### 2.4.1. Derivatives and Differentiation
Put simply, a derivative is the rate of change in a function with respect to changes in its arguments. Derivatives can tell us how rapidly a loss function would increase or decrease were we to increase or decrease each parameter by an infinitesimally small amount. Formally, for functions $f: \mathbb{R} \rightarrow \mathbb{R}$, the derivative of $f$ is defined as

$$f'(x) = \lim_{h \rightarrow 0} \frac{f(x+h) - f(x)}{h}.$$

The notation for derivatives varies quite a bit. The following are all common ways of expressing the same thing:

$$\frac{df(x)}{dx} = f'(x) = D_x f(x) = D_x y = y' = \frac{dy}{dx}.$$

The derivative of a function $f$ with respect to its argument $x$ is defined as

$$f'(x) = \lim_{h \rightarrow 0} \frac{f(x+h) - f(x)}{h}.$$

In deep learning, our models are parameterized by sets of weights and biases. We often want to optimize these parameters to minimize some loss function, e.g., to improve our model’s accuracy on held-out data or to minimize the production cost. In order to do so, we will need to take derivatives of these loss functions with respect to our model parameters. Thus it is worth reviewing the basics of derivatives.

### 2.4.2. Partial Derivatives
When our functions depend on more than one variable, we call them multivariate functions. Given a multivariate function $f(\mathbf{x}) : \mathbb{R}^n \rightarrow \mathbb{R}$, its partial derivative with respect to $x_i$ is

$$\frac{\partial}{\partial x_i} f(\mathbf{x}) = \lim_{h \rightarrow 0} \frac{f(\mathbf{x} + h \mathbf{e}_i) - f(\mathbf{x})}{h},$$

where $\mathbf{e}_i$ is a vector of zeros with a one in the $i^\mathrm{th}$ coordinate. In multivariate functions, we can take derivatives with respect to any variable, treating all others as constants. This is called partial differentiation. To calculate a partial derivative, we simply differentiate as usual, following all the usual rules, but treating all the other variables as constants.

### 2.4.3. Gradients

The gradient of a multivariate function $f: \mathbb{R}^n \rightarrow \mathbb{R}$ is the vector containing all of the partial derivatives, denoted $\nabla_{\mathbf{x}} f(\mathbf{x})$:

$$\nabla_{\mathbf{x}} f(\mathbf{x}) = \begin{bmatrix} \frac{\partial}{\partial x_1} f(\mathbf{x})  \\ \frac{\partial}{\partial x_2} f(\mathbf{x}) \\ \vdots \\ \frac{\partial}{\partial x_n} f(\mathbf{x}) \end{bmatrix}.$$

The gradient points in the direction of the greatest rate of increase of the function $f$, and its magnitude is the slope of the function in that direction. The gradient of a function $f: \mathbb{R}^n \rightarrow \mathbb{R}$ gives us a vector $\nabla_{\mathbf{x}} f(\mathbf{x})$ whose entries are the partial derivatives of $f$ with respect to each of the input variables. This vector points in the direction of greatest increase of $f$ and its magnitude is the slope in that direction.

### 2.4.4. Chain Rule

The chain rule allows us to differentiate compositions of functions. Given two functions $f: \mathbb{R}^m \rightarrow \mathbb{R}^n$ and $g: \mathbb{R}^n \rightarrow \mathbb{R}^p$, the chain rule states that the gradient of the composition $g \circ f$ is given by

$$\nabla (\mathbf{g} \circ \mathbf{f})(\mathbf{x}) = \mathbf{J}_{\mathbf{f}}(\mathbf{x})^\top \mathbf{J}_{\mathbf{g}}(\mathbf{f}(\mathbf{x})),$$

where $\mathbf{J}_{\mathbf{f}}(\mathbf{x})$ and $\mathbf{J}_{\mathbf{g}}(\mathbf{f}(\mathbf{x}))$ are the Jacobian matrices of $f$ and $g$, respectively. In other words, the gradient of the function composition is the matrix product of the two Jacobian matrices.

### 2.4.5. Discussion
While we have just scratched the surface of a deep topic, a number of concepts already come into focus: 
- First, the composition rules for differentiation can be applied routinely, enabling us to compute gradients automatically. This task requires no creativity and thus we can focus our cognitive powers elsewhere. 
- Second, computing the derivatives of vector-valued functions requires us to multiply matrices as we trace the dependency graph of variables from output to input. In particular, this graph is traversed in a forward direction when we evaluate a function and in a backwards direction when we compute gradients. Later chapters will formally introduce backpropagation, a computational procedure for applying the chain rule.

From the viewpoint of optimization, gradients allow us to determine how to move the parameters of a model in order to lower the loss, and each step of the optimization algorithms used throughout this book will require calculating the gradient.

## 2.5 Automatic Differentiation

### 2.5.1. A Simple Example

To get started we import the `autograd` package. Here, `autograd` stands for automatic differentiation.

=== "PYTORCH"

    ```python
    x = torch.arange(4.0)
    x
    ```

    ```text
    tensor([0., 1., 2., 3.], requires_grad=True)
    ```
=== "TENSORFLOW"

    ```python
    x = tf.range(4.0)
    x
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([0., 1., 2., 3.], dtype=float32)>
    ```
Before we can compute gradients, we need to allocate memory for the gradients we wish to calculate.

=== "PYTORCH"

    ```python
    x.requires_grad_(True)  # Same as `x = torch.arange(4.0, requires_grad=True)`
    x.grad  # The default value is None
    ```

    ```text
    tensor([0., 1., 2., 3.], requires_grad=True)
    ```
=== "TENSORFLOW"

    ```python
    x = tf.Variable(x)
    x
    ```

    ```text
    <tf.Variable 'Variable:0' shape=(4,) dtype=float32, numpy=array([0., 1., 2., 3.], dtype=float32)>
    ```
Now let's compute the sum of the squares of `x`.

=== "PYTORCH"

    ```python
    y = 2 * torch.dot(x, x)
    y
    ```

    ```text
    tensor(28., grad_fn=<MulBackward0>)
    ```
=== "TENSORFLOW"

    ```python
    y = 2 * tf.tensordot(x, x, axes=1)
    y
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=28.0>
    ```
We can now take the gradient of y with respect to x by calling its backward method. Next, we can access the gradient via x’s grad attribute.

=== "PYTORCH"

    ```python
    y.backward()
    x.grad
    ```

    ```text
    tensor([ 0.,  4.,  8., 12.])
    ```
=== "TENSORFLOW"

    ```python
    with tf.GradientTape() as t:
        y = 2 * tf.tensordot(x, x, axes=1)
    x_grad = t.gradient(y, x)
    x_grad
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 0.,  4.,  8., 12.], dtype=float32)>
    ```

### 2.5.2. Backward for Non-Scalar Variables

When y is a vector, the most natural representation of the derivative of y with respect to a vector x is a matrix called the Jacobian that contains the partial derivatives of each component of y with respect to each component of x. Likewise, for higher-order y and x, the result of differentiation could be an even higher-order tensor.

While Jacobians do show up in some advanced machine learning techniques, more commonly we want to sum up the gradients of each component of y with respect to the full vector x, yielding a vector of the same shape as x. For example, we often have a vector representing the value of our loss function calculated separately for each example among a batch of training examples. Here, we just want to sum up the gradients computed individually for each example.

=== "PYTORCH"

    ```python
    x.grad.zero_()
    y = x.sum()
    y.backward()
    x.grad
    ```

    ```text
    tensor([1., 1., 1., 1.])
    ```
=== "TENSORFLOW"

    ```python
    with tf.GradientTape() as t:
        y = tf.reduce_sum(x)
    x_grad = t.gradient(y, x)
    x_grad
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([1., 1., 1., 1.], dtype=float32)>
    ```
### 2.5.3. Detaching Computation

Sometimes, we wish to move some calculations outside of the recorded computational graph. For example, say that y is a function of x, and that we wish to compute the gradient of y with respect to x, but that we plan to use the value of y, say y_val later on, without ever computing further derivatives of y with respect to x.

=== "PYTORCH"

    ```python
    x.grad.zero_()
    y = x * x
    u = y.detach()
    z = u * x

    z.sum().backward()
    x.grad == u
    ```

    ```text
    tensor([True, True, True, True])
    ```
=== "TENSORFLOW"

    ```python
    # Set persistent=True to preserve the compute graph.
    # This lets us run t.gradient more than once
    with tf.GradientTape(persistent=True) as t:
        y = x * x
        u = tf.stop_gradient(y)
        z = u * x

    x_grad = t.gradient(z, x)
    x_grad == u
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=bool, numpy=array([ True,  True,  True,  True])>
    ```

=== "PYTORCH"

    ```python
    x.grad.zero_()
    y.sum().backward()
    x.grad == 2 * x
    ```

    ```text
    tensor([True, True, True, True])
    ```
=== "TENSORFLOW"

    ```python
    x_grad = t.gradient(y.sum(), x)
    x_grad == 2 * x
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=bool, numpy=array([ True,  True,  True,  True])>
    ```

### 2.5.4. Computing the Gradient of Python Control Flow

One benefit of using automatic differentiation is that even if the computational graph of the function contains Python’s control flow (such as conditional and loops), we can still calculate the gradient of the resulting variable. Consider the following program:

=== "PYTORCH"

    ```python
    def f(a):
        b = a * 2
        while b.norm() < 1000:
            b = b * 2
        if b.sum() > 0:
            c = b
        else:
            c = 100 * b
        return c
    ```
=== "TENSORFLOW"

    ```python
    def f(a):
        b = a * 2
        while tf.norm(b) < 1000:
            b = b * 2
        if tf.reduce_sum(b) > 0:
            c = b
        else:
            c = 100 * b
        return c
    ```
Let us compute the gradient.

=== "PYTORCH"

    ```python
    a = torch.randn(size=(), requires_grad=True)
    d = f(a)
    d.backward()
    a.grad == d / a
    ```

    ```text
    tensor(True)
    ```
=== "TENSORFLOW"

    ```python
    a = tf.Variable(tf.random.normal(shape=()))
    with tf.GradientTape() as t:
        d = f(a)
    d_grad = t.gradient(d, a)
    d_grad == d / a
    ```

    ```text
    <tf.Tensor: shape=(), dtype=bool, numpy=True>
    ```
### 2.5.5. Discussion

You have now gotten a taste of the power of automatic differentiation. The development of libraries for calculating derivatives both automatically and efficiently has been a massive productivity booster for deep learning practitioners, liberating them so they can focus on less menial. Moreover, autograd lets us design massive models for which pen and paper gradient computations would be prohibitively time consuming. Interestingly, while we use autograd to optimize models (in a statistical sense) the optimization of autograd libraries themselves (in a computational sense) is a rich subject of vital interest to framework designers. Here, tools from compilers and graph manipulation are leveraged to compute results in the most expedient and memory-efficient manner.

For now, try to remember these basics: 
(i) attach gradients to those variables with respect to which we desire derivatives; (ii) record the computation of the target value; 
(iii) execute the backpropagation function;
(iv) access the resulting gradient.

## 2.6 Probability and Statistics

One way or another, machine learning is all about uncertainty. In supervised learning, we want to predict something unknown (the target) given something known (the features). Depending on our objective, we might attempt to predict the most likely value of the target. Or we might predict the value with the smallest expected distance from the target. And sometimes we wish not only to predict a specific value but to quantify our uncertainty. For example, given some features describing a patient, we might want to know how likely they are to suffer a heart attack in the next year. In unsupervised learning, we often care about uncertainty. To determine whether a set of measurements are anomalous, it helps to know how likely one is to observe values in a population of interest. Furthermore, in reinforcement learning, we wish to develop agents that act intelligently in various environments. This requires reasoning about how an environment might be expected to change and what rewards one might expect to encounter in response to each of the available actions.

Probability is the mathematical field concerned with reasoning under uncertainty. Given a probabilistic model of some process, we can reason about the likelihood of various events. The use of probabilities to describe the frequencies of repeatable events (like coin tosses) is fairly uncontroversial. In fact, **frequentist** scholars adhere to an interpretation of probability that applies only to such repeatable events. By contrast **Bayesian** scholars use the language of probability more broadly to formalize reasoning under uncertainty. Bayesian probability is characterized by two unique features: **(i) assigning degrees of belief to non-repeatable events, e.g., what is the probability that a dam will collapse?; and (ii) subjectivity.** While Bayesian probability provides unambiguous rules for how one should update their beliefs in light of new evidence, it allows for different individuals to start off with different prior beliefs. Statistics helps us to reason backwards, starting off with collection and organization of data and backing out to what inferences we might draw about the process that generated the data. Whenever we analyze a dataset, hunting for patterns that we hope might characterize a broader population, we are employing statistical thinking. Many courses, majors, theses, careers, departments, companies, and institutions have been devoted to the study of probability and statistics. While this section only scratches the surface, we will provide the foundation that you need to begin building models.

### 2.6.1. Basics

### 2.6.2 Advanced Topics

### 2.6.3 Random Variables

### 2.6.4 Multile Random Variables

### 2.6.5 Expectation

### 2.6.6 Variance

### 2.6.7 Summary

## 2.7 Documentation

### 2.7.1. Functions and Classes in a Module

### 2.7.2. Specific Functions and Classes

