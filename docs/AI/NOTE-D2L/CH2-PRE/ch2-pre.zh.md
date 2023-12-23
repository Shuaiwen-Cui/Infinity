# CH02 - 预备知识

要学习深度学习，首先需要先掌握一些基本技能。 所有机器学习方法都涉及从数据中提取信息。 因此，我们先学习一些关于数据的实用技能，包括存储、操作和预处理数据。

机器学习通常需要处理大型数据集。 我们可以将某些数据集视为一个表，其中表的行对应样本，列对应属性。 线性代数为人们提供了一些用来处理表格数据的方法。 我们不会太深究细节，而是将重点放在矩阵运算的基本原理及其实现上。

深度学习是关于优化的学习。 对于一个带有参数的模型，我们想要找到其中能拟合数据的最好模型。 在算法的每个步骤中，决定以何种方式调整参数需要一点微积分知识。 本章将简要介绍这些知识。 幸运的是，autograd包会自动计算微分，本章也将介绍它。

机器学习还涉及如何做出预测：给定观察到的信息，某些未知属性可能的值是多少？ 要在不确定的情况下进行严格的推断，我们需要借用概率语言。

最后，官方文档提供了本书之外的大量描述和示例。 在本章的结尾，我们将展示如何在官方文档中查找所需信息。

## 2.1 数据操作

### 2.1.1 入门

导入`torch`或`tensorflow`库

=== "PYTORCH"

    ```python
    import torch
    ```
    
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    ```
张量是一个数组，它可以具有任意数量的维度。 例如，向量是一维张量，矩阵是二维张量。 张量中的每个元素都具有相同的数据类型，且张量的形状由每个维度的大小组成。

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
如果想要查看张量的形状，可以访问其`shape`属性。

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
如果想要查看张量中元素的总数，可以调用`numel`函数或者`size`属性。

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
    12
    ```
如果想要改变张量形状，而不改变元素数量和元素值，可以调用`reshape`函数。 例如，我们可以将张量`x`从形状为(12,)的一维张量转换为形状为(3,4)的二维张量。

=== "PYTORCH"

    ```python
    x.reshape(3, 4)
    ```

    ```text
    tensor([[ 0,  1,  2,  3],
            [ 4,  5,  6,  7],
            [ 8,  9, 10, 11]])
    ```
=== "TENSORFLOW"

    ```python
    tf.reshape(x, (3, 4))
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=int32, numpy=
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11]], dtype=int32)>
    ```
如果我们只知道想要张量具有多少列，而不知道想要张量具有多少行，我们可以用-1指定，让系统自动推断出此值。 在上面的例子中，由于张量`x`包含12个元素，因此我们可以用`x.reshape(-1, 4)`或`x.reshape(3, -1)`来重塑张量`x`为形状为(3,4)或(4,3)的张量。

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
    tf.reshape(x, (3, -1))
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=int32, numpy=
    array([[ 0,  1,  2,  3],
           [ 4,  5,  6,  7],
           [ 8,  9, 10, 11]], dtype=int32)>
    ```
有时，我们希望使用全0、全1、其他常量或者从特定分布中随机采样的数字张量来初始化张量。 我们可以创建一个形状为(2,3,4)的张量，其中所有元素都设置为0。

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

同样，我们可以创建一个形状为(2,3,4)的张量，其中所有元素都设置为1。

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
我们也可以通过Python的列表（或嵌套列表）指定需要创建的张量中每个元素的值。

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
通常，我们希望从某个概率分布中随机采样一些值，例如在深度学习中经常使用的正态分布。 在本书中，我们不需要深入了解概率和统计知识。 有关这些主题的优秀书籍包括概率导论 [@blitzstein2014introduction] 和统计学习方法 [@lihang2012statistical]。 为了生成具有形状为(3,4)的张量。 其中的每个元素都将随机采样于均值为0、标准差为1的正态分布。

=== "PYTORCH"

    ```python
    torch.randn(3, 4)
    ```

    ```text
    tensor([[ 0.1835,  0.7694, -0.4696,  0.5421],
            [-0.4637, -0.0328,  0.2369, -0.4107],
            [-0.9880, -0.5176, -0.1713, -0.2025]])
    ```
=== "TENSORFLOW"

    ```python
    tf.random.normal((3, 4))
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=float32, numpy=
    array([[ 0.1835,  0.7694, -0.4696,  0.5421],
           [-0.4637, -0.0328,  0.2369, -0.4107],
           [-0.9880, -0.5176, -0.1713, -0.2025]], dtype=float32)>
    ```

### 2.1.2 运算符

张量支持大量的运算符（操作符）。 例如，我们可以对两个张量按元素相加。 给定张量`X`和`Y`，我们可以使用`X + Y`来实现按元素加法。 对于按元素乘法、按元素除法和按元素指数运算，我们分别可以使用`X * Y`、`X / Y`和`X**Y`。 符号`**`代表按元素乘方运算。对于任意具有相同形状的张量， 常见的标准算术运算符（+、-、*、/和**）都可以被升级为按元素运算。 我们可以在同一形状的任意两个张量上调用按元素操作。 在下面的例子中，我们使用逗号来表示一个具有5个元素的元组，其中每个元素都是按元素操作的结果。

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
“按元素”方式可以应用更多的计算，包括像求幂这样的一元运算符。

=== "PYTORCH"

    ```python
    torch.exp(x)
    ```

    ```text
    tensor([  2.7183,   7.3891,  54.5981, 298.8674])
    ```
=== "TENSORFLOW"

    ```python
    tf.exp(x)
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([  2.7183,   7.3891,  54.5981, 298.8674], dtype=float32)>
    ```
我们也可以把多个张量连结（concatenate）在一起， 把它们端对端地叠起来形成一个更大的张量。 我们只需要提供张量列表，并给出沿哪个轴连结。

=== "PYTORCH"

    ```python
    X = torch.arange(12, dtype=torch.float32).reshape((3, 4))
    Y = torch.tensor([[2.0, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
    torch.cat((X, Y), dim=0), torch.cat((X, Y), dim=1)
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
    X = tf.reshape(tf.range(12, dtype=tf.float32), (3, 4))
    Y = tf.constant([[2.0, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
    tf.concat((X, Y), axis=0), tf.concat((X, Y), axis=1)
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
有时，我们想通过逻辑运算符构建二元张量。

=== "PYTORCH"

    ```python
    X == Y
    ```

    ```text
    tensor([[False,  True, False,  True],
            [False, False, False, False],
            [False, False, False, False]])
    ```
=== "TENSORFLOW"

    ```python
    X == Y
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=bool, numpy=
    array([[False,  True, False,  True],
           [False, False, False, False],
           [False, False, False, False]])>
    ```
对张量中的所有元素进行求和会产生一个只有一个元素的张量。

=== "PYTORCH"

    ```python
    X.sum()
    ```

    ```text
    tensor(66.)
    ```
=== "TENSORFLOW"

    ```python
    tf.reduce_sum(X)
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=66.0>
    ```
### 2.1.3 广播机制

在上面的部分中，我们看到了如何在相同形状的两个张量上执行按元素操作。 在某些情况下，我们还可以在不同形状的张量上执行按元素操作。 当两个张量的形状不同时，我们将通过广播机制（broadcasting mechanism）来执行按元素操作。 这个机制的工作方式如下： 首先，通过适当复制元素来扩展一个或两个数组，以便在转换之后，两个张量具有相同的形状。 其次，对生成的数组执行按元素操作。

在大多数情况下，我们将沿着数组中长度为1的轴进行广播，如下例所示。

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
由于`a`和`b`分别是3行1列和1行2列的矩阵，如果我们想要将它们相加，那么`a`中第一列的3个元素被广播（复制）到了第二列，而`b`中第一行的2个元素被广播（复制）到了第二行和第三行。 如此，我们就可以对两个3行2列的矩阵按元素相加。

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

### 2.1.4 索引和切片

就像在任何其他Python数组中一样，张量中的元素可以通过索引访问。 与任何Python数组一样，第一个元素的索引是0，而不是1。 与标准Python列表一样，我们可以通过使用方括号（[]）来访问和分配张量中的元素。

=== "PYTORCH"

    ```python
    X[-1], X[1:3]
    ```

    ```text
    (tensor([ 8.,  9., 10., 11.]), tensor([[ 4.,  5.,  6.,  7.],
             [ 8.,  9., 10., 11.]]))
    ```
=== "TENSORFLOW"

    ```python
    X[-1], X[1:3]
    ```

    ```text
    (<tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 8.,  9., 10., 11.], dtype=float32)>,
     <tf.Tensor: shape=(2, 4), dtype=float32, numpy=
    array([[ 4.,  5.,  6.,  7.],
           [ 8.,  9., 10., 11.]], dtype=float32)>)
    ```

除读取外，我们还可以通过指定索引来将元素写入矩阵。

=== "PYTORCH"

    ```python
    X[1, 2] = 9
    X
    ```

    ```text
    tensor([[ 0.,  1.,  2.,  3.],
            [ 4.,  5.,  9.,  7.],
            [ 8.,  9., 10., 11.]])
    ```
=== "TENSORFLOW"

    ```python
    X[1, 2].assign(9)
    X
    ```

    ```text 
    <tf.Tensor: shape=(3, 4), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.],
           [ 4.,  5.,  9.,  7.],
           [ 8.,  9., 10., 11.]], dtype=float32)>
    ```

如果我们想为多个元素赋值相同的值，我们只需要索引所有元素，然后为它们赋值。 例如，`[0:2, :]`访问第1行和第2行，其中“:”代表沿轴0（行）的所有元素。 虽然我们讨论的是矩阵的索引，但这也适用于向量和超过2个维度的张量。

=== "PYTORCH"

    ```python
    X[0:2, :] = 12
    X
    ```

    ```text
    tensor([[12., 12., 12., 12.],
            [12., 12., 12., 12.],
            [ 8.,  9., 10., 11.]])
    ```
=== "TENSORFLOW"

    ```python
    X[0:2, :].assign(12)
    X
    ```

    ```text
    <tf.Tensor: shape=(3, 4), dtype=float32, numpy=
    array([[12., 12., 12., 12.],
           [12., 12., 12., 12.],
           [ 8.,  9., 10., 11.]], dtype=float32)>
    ```
### 2.1.5 节省内存

运行一些操作可能会导致为新结果分配内存。 例如，如果我们用`Y = X + Y`，我们会将`Y`指向新创建的内存，而不是将`Y`指向`X`的原始位置。 我们可以使用Python的`id`函数来验证这一点，这个函数给我们提供了内存中引用对象的确切地址。 运行`Y = Y + X`后，我们会发现`id(Y)`指向另一个位置。 这是因为Python首先计算`Y + X`，为结果分配新的内存，然后使`Y`指向内存中的这个新位置。

=== "PYTORCH"

    ```python
    before = id(Y)
    Y = Y + X
    id(Y) == before
    ```

    ```text
    False
    ```
=== "TENSORFLOW"

    ```python
    before = id(Y)
    Y = Y + X
    id(Y) == before
    ```

    ```text
    False
    ```
这可能是不可取的，原因有两个：

首先，我们不想总是不必要地分配内存。在机器学习中，我们可能有数百兆的参数，并且在一秒内多次更新所有参数。通常情况下，我们希望原地执行这些更新；

如果我们不原地更新，其他引用仍然会指向旧的内存位置，这样我们的某些代码可能会无意中引用旧的参数。

幸运的是，(**执行原地操作**)的方法很简单。我们可以使用切片表示法将操作的结果分配给先前分配给结果的数组，例如`Y[:] = <expression>`。为了说明这一点，我们首先创建一个新的矩阵`Z`，其形状与另一个`Y`相同，使用`zeros_like`来分配一个全0的块。

=== "PYTORCH"

    ```python
    Z = torch.zeros_like(Y)
    print('id(Z):', id(Z))
    Z[:] = X + Y
    print('id(Z):', id(Z))
    ```

    ```text
    id(Z): 140703086993984
    id(Z): 140703086993984
    ```
=== "TENSORFLOW"

    ```python
    Z = tf.zeros_like(Y)
    print('id(Z):', id(Z))
    Z[:] = X + Y
    print('id(Z):', id(Z))
    ```

    ```text
    id(Z): 140703086993984
    id(Z): 140703086993984
    ```
现在，我们用`X + Y`替换`Y`。我们使用`[:]`来指定我们想要用分配给`X + Y`的结果来更新`Y`中的值。

=== "PYTORCH"

    ```python
    before = id(Y)
    Y[:] = X + Y
    id(Y) == before
    ```

    ```text
    True
    ```
=== "TENSORFLOW"

    ```python
    before = id(Y)
    Y[:] = X + Y
    id(Y) == before
    ```

    ```text
    True
    ```
### 2.1.6 转换为其他 Python 对象

将深度学习框架定义的张量转换为NumPy张量（ndarray）很容易，反之也同样容易。 转换后的结果不共享内存。 这个小的不便实际上是非常重要的：当在CPU或GPU上执行操作的时候， 如果Python的NumPy包也希望使用相同的内存块执行其他操作，人们不希望停下计算来等它。

=== "PYTORCH"

    ```python
    A = X.numpy()
    B = torch.tensor(A)
    type(A), type(B)
    ```

    ```text
    (numpy.ndarray, torch.Tensor)
    ```
=== "TENSORFLOW"

    ```python
    A = X.numpy()
    B = tf.convert_to_tensor(A)
    type(A), type(B)
    ```

    ```text
    (numpy.ndarray, tensorflow.python.framework.ops.EagerTensor)
    ```
要(**将大小为1的张量转换为Python标量**)，我们可以调用`item`函数或Python的内置函数。

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
### 2.1.7 小结

深度学习存储和操作数据的主要接口是张量（n维数组）。它提供了各种功能，包括基本数学运算、广播、索引、切片、内存节省和转换其他Python对象。











## 2.2 数据预处理
pandas可以与张量兼容

- 读取数据集
- 处理缺失值
- 转换为张量格式
- 小结

## 2.3 线性代数

## 2.4 微积分

## 2.5 微分

## 2.6 概率与统计

## 2.7 文档

