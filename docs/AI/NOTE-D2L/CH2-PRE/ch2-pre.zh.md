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
为了能用深度学习来解决现实世界的问题，我们经常从预处理原始数据开始， 而不是从那些准备好的张量格式数据开始。 在Python中常用的数据分析工具中，我们通常使用pandas软件包。 像庞大的Python生态系统中的许多其他扩展包一样，pandas可以与张量兼容。 本节我们将简要介绍使用pandas预处理原始数据，并将原始数据转换为张量格式的步骤。 后面的章节将介绍更多的数据预处理技术。

### 2.2.1 读取数据集
举一个例子，我们首先创建一个人工数据集，并存储在CSV（逗号分隔值）文件 ../data/house_tiny.csv中。 以其他格式存储的数据也可以通过类似的方式进行处理。 下面我们将数据集按行写入CSV文件中。

```python
import os

os.makedirs(os.path.join('..', 'data'), exist_ok=True)
data_file = os.path.join('..', 'data', 'house_tiny.csv')
with open(data_file, 'w') as f:
    f.write('NumRooms,Alley,Price\n')  # 列名
    f.write('NA,Pave,127500\n')  # 每行表示一个数据样本
    f.write('2,NA,106000\n')
    f.write('4,NA,178100\n')
    f.write('NA,NA,140000\n')
```
要从创建的CSV文件中加载原始数据集，我们导入pandas包并调用read_csv函数。该数据集有四行三列。其中每行描述了房间数量（“NumRooms”）、巷子类型（“Alley”）和房屋价格（“Price”）。

=== "PYTORCH"

    ```python
    !pip install pandas
    import pandas as pd
    data = pd.read_csv(data_file)
    data
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
    !pip install pandas
    import pandas as pd
    data = pd.read_csv(data_file)
    data
    ```

    ```text
       NumRooms Alley   Price
    0       NaN  Pave  127500
    1       2.0   NaN  106000
    2       4.0   NaN  178100
    3       NaN   NaN  140000
    ```
### 2.2.2 处理缺失值

注意，“NaN”项代表缺失值。 为了处理缺失的数据，典型的方法包括插值法和删除法， 其中插值法用一个替代值弥补缺失值，而删除法则直接忽略缺失值。 在这里，我们将考虑插值法。

通过位置索引iloc，我们将data分成inputs和outputs， 其中前者为data的前两列，而后者为data的最后一列。 对于inputs中缺少的数值，我们用同一列的均值替换“NaN”项。

=== "PYTORCH"

    ```python
    inputs, outputs = data.iloc[:, 0:2], data.iloc[:, 2]
    inputs = inputs.fillna(inputs.mean())
    inputs
    ```

    ```text
       NumRooms Alley
    0       3.0  Pave
    1       2.0   NaN
    2       4.0   NaN
    3       3.0   NaN
    ```
=== "TENSORFLOW"

    ```python
    inputs, outputs = data.iloc[:, 0:2], data.iloc[:, 2]
    inputs = inputs.fillna(inputs.mean())
    inputs
    ```

    ```text
       NumRooms Alley
    0       3.0  Pave
    1       2.0   NaN
    2       4.0   NaN
    3       3.0   NaN
    ```
对于inputs中的类别值或离散值，我们将“NaN”视为一个类别。 由于“巷子类型”（“Alley”）列只接受两种类型的类别值“Pave”和“NaN”， pandas可以自动将此列转换为两列“Alley_Pave”和“Alley_nan”。 巷子类型为“Pave”的行会将“Alley_Pave”的值设置为1，“Alley_nan”的值设置为0。 缺少巷子类型的行会将“Alley_Pave”和“Alley_nan”分别设置为0和1。

=== "PYTORCH"

    ```python
    inputs = pd.get_dummies(inputs, dummy_na=True)
    inputs
    ```

    ```text
       NumRooms  Alley_Pave  Alley_nan
    0       3.0           1          0
    1       2.0           0          1
    2       4.0           0          1
    3       3.0           0          1
    ```
=== "TENSORFLOW"

    ```python
    inputs = pd.get_dummies(inputs, dummy_na=True)
    inputs
    ```

    ```text
       NumRooms  Alley_Pave  Alley_nan
    0       3.0           1          0
    1       2.0           0          1
    2       4.0           0          1
    3       3.0           0          1
    ```

### 2.2.3 转换为张量格式

现在inputs和outputs中的所有条目都是数值类型，它们可以转换为张量格式。 当数据采用张量格式后，我们可以(**通过在 :numref:`sec_ndarray` 中引入的张量逻辑来(**(**访问任意元素。**)**)

=== "PYTORCH"

    ```python
    import torch
    X, y = torch.tensor(inputs.values), torch.tensor(outputs.values)
    X, y
    ```

    ```text
    (tensor([[3., 1., 0.],
             [2., 0., 1.],
             [4., 0., 1.],
             [3., 0., 1.]], dtype=torch.float64),
     tensor([127500, 106000, 178100, 140000]))
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    X, y = tf.constant(inputs.values), tf.constant(outputs.values)
    X, y
    ```

    ```text
    (<tf.Tensor: shape=(4, 3), dtype=float64, numpy=
    array([[3., 1., 0.],
           [2., 0., 1.],
           [4., 0., 1.],
           [3., 0., 1.]])>,
     <tf.Tensor: shape=(4,), dtype=int64, numpy=array([127500, 106000, 178100, 140000])>)
    ```
### 2.2.4 小结
- pandas软件包是Python中常用的数据分析工具中，pandas可以与张量兼容。

- 用pandas处理缺失的数据时，我们可根据情况选择用插值法和删除法。

## 2.3 线性代数
在介绍完如何存储和操作数据后，接下来将简要地回顾一下部分基本线性代数内容。 这些内容有助于读者了解和实现本书中介绍的大多数模型。 本节将介绍线性代数中的基本数学对象、算术和运算，并用数学符号和相应的代码实现来表示它们。

### 2.3.1 标量
标量由只有一个元素的张量表示。 下面的代码将实例化两个标量，并执行一些熟悉的算术运算，即加法、乘法、除法和指数。

=== "PYTORCH"

    ```python
    import torch
    x = torch.tensor(3.0)
    y = torch.tensor(2.0)

    x + y, x * y, x / y, x**y
    ```

    ```text
    (tensor([5.]), tensor([6.]), tensor([1.5000]), tensor([9.]))
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    x = tf.constant(3.0)
    y = tf.constant(2.0)

    x + y, x * y, x / y, x**y
    ```

    ```text
    (<tf.Tensor: shape=(1,), dtype=float32, numpy=array([5.], dtype=float32)>,
     <tf.Tensor: shape=(1,), dtype=float32, numpy=array([6.], dtype=float32)>,
     <tf.Tensor: shape=(1,), dtype=float32, numpy=array([1.5], dtype=float32)>,
     <tf.Tensor: shape=(1,), dtype=float32, numpy=array([9.], dtype=float32)>)
    ```
### 2.3.2 向量
向量可以被视为标量值组成的列表。 这些标量值被称为向量的元素（element）或分量（component）。 当向量表示数据集中的样本时，它们的值具有一定的现实意义。在数学表示法中，我们通常用粗体、小写字母表示向量。 

=== "PYTORCH"

    ```python
    import torch
    x = torch.arange(4)
    x
    ```

    ```text
    tensor([0, 1, 2, 3])
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    x = tf.range(4)
    x
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=int32, numpy=array([0, 1, 2, 3], dtype=int32)>
    ```
我们可以使用下标来引用向量的任一元素.

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
#### 2.3.2.1 长度、维度和形状
向量只是一个数字数组，就像每个数组都有一个长度一样，每个向量也是如此。 向量的长度通常称为维度。

与普通的Python数组一样，我们可以通过调用Python的内置len()函数来访问张量的长度。

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
当用张量表示一个向量（只有一个轴）时，我们也可以通过.shape属性访问向量的长度。 形状（shape）是一个元素组，列出了张量沿每个轴的长度（维数）。 对于只有一个轴的张量，形状只有一个元素。

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
请注意，维度（dimension）这个词在不同上下文时往往会有不同的含义，这经常会使人感到困惑。 为了清楚起见，我们在此明确一下： 向量或轴的维度被用来表示向量或轴的长度，即向量或轴的元素数量。 然而，张量的维度用来表示张量具有的轴数。 在这个意义上，张量的某个轴的维数就是这个轴的长度。

### 2.3.3 矩阵

正如向量将标量从零阶推广到一阶，矩阵将向量从一阶推广到二阶。矩阵，在代码中表示为具有两个轴的张量。当调用函数来实例化张量时， 我们可以通过指定两个分量m和n来创建一个形状为mxn的矩阵。

=== "PYTORCH"

    ```python
    import torch
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
    import tensorflow as tf
    A = tf.reshape(tf.range(20), (5, 4))
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
我们可以通过行索引（row）和列索引（column）来访问矩阵中的标量元素。 为了表示矩阵`A`中行索引为`i`、列索引为`j`的元素，我们通常使用标量记法$a_{i,j}$。

当我们交换矩阵的行和列时，结果称为矩阵的转置（transpose）。

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
### 2.3.4 张量
就像向量是标量的推广，矩阵是向量的推广一样，我们可以构建具有更多轴的数据结构。 张量（本小节中的“张量”指代数对象）是描述具有任意数量轴的n维数组的通用方法。 例如，向量是一阶张量，矩阵是二阶张量。

=== "PYTORCH"

    ```python
    import torch
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
    import tensorflow as tf
    X = tf.reshape(tf.range(24), (2, 3, 4))
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
张量保留了矩阵的所有性质。 张量具有轴、秩和形状。 一个张量可以通过指定它们的轴数来指定轴的数量。 张量的轴数也称为它的阶（order）、秩（rank）或ndim（表示“number of dimensions”）。 矩阵有两个轴（行和列），因此我们说它的阶为2，它的秩为2，它的ndim为2。 同样，我们可以说，张量的阶、秩或ndim等于它的轴数。

### 2.3.5 张量算法的基本性质
标量、向量、矩阵和任意数量轴的张量（本小节中的“张量”指代数对象）有一些实用的属性。 例如，从按元素操作的定义中可以注意到，任何按元素的一元运算都不会改变其操作数的形状。 同样，给定具有相同形状的任意两个张量，任何按元素二元运算的结果都将是相同形状的张量。 例如，将两个相同形状的矩阵相加，会在这两个矩阵上执行元素加法。

=== "PYTORCH"

    ```python
    A = torch.arange(20, dtype=torch.float32).reshape(5, 4)
    B = A.clone()  # 通过分配新内存，将A的一个副本分配给B
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
    A = tf.reshape(tf.range(20, dtype=tf.float32), (5, 4))
    B = A  # 不能通过分配新内存将A克隆到B
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
具体而言，两个矩阵按元素乘法称为哈达玛积（Hadamard product）（数学符号$\odot$），而不是矩阵乘法。 两个矩阵按元素相乘的结果与两个矩阵的形状相同。

=== "PYTORCH"

    ```python
    A * B
    ```

    ```text
    tensor([[  0.,   1.,   4.,   9.],
            [ 16.,  25.,  36.,  49.],
            [ 64.,  81., 100., 121.],
            [144., 169., 196., 225.],
            [256., 289., 324., 361.]])
    ```
=== "TENSORFLOW"

    ```python
    A * B
    ```

    ```text
    <tf.Tensor: shape=(5, 4), dtype=float32, numpy=
    array([[  0.,   1.,   4.,   9.],
           [ 16.,  25.,  36.,  49.],
           [ 64.,  81., 100., 121.],
           [144., 169., 196., 225.],
           [256., 289., 324., 361.]], dtype=float32)>
    ```
### 2.3.6 降维

我们可以对任意张量进行的一个有用的操作是计算其元素的和。在代码中可以调用计算求和的函数：

=== "PYTORCH"

    ```python
    import torch
    x = torch.arange(4, dtype=torch.float32)
    x, x.sum()
    ```

    ```text
    (tensor([0., 1., 2., 3.]), tensor(6.))
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    x = tf.range(4, dtype=tf.float32)
    x, tf.reduce_sum(x)
    ```

    ```text
    (<tf.Tensor: shape=(4,), dtype=float32, numpy=array([0., 1., 2., 3.], dtype=float32)>,
     <tf.Tensor: shape=(), dtype=float32, numpy=6.0>)
    ```
默认情况下，调用求和函数会沿所有的轴降低张量的维度，使它变为一个标量。 我们还可以指定张量沿哪一个轴来通过求和降低维度。 以矩阵为例，为了通过求和所有行的元素来降维（轴0），可以在调用函数时指定axis=0。 由于输入矩阵沿0轴降维以生成输出向量，因此输入轴0的维数在输出形状中消失。

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
    (<tf.Tensor: shape=(4,), dtype=float32, numpy=array([40., 45., 50., 55.], dtype=float32)>, TensorShape([4]))
    ```
默认情况下，调用求和函数会沿所有的轴降低张量的维度，使它变为一个标量。 我们还可以指定张量沿哪一个轴来通过求和降低维度。 以矩阵为例，为了通过求和所有行的元素来降维（轴0），可以在调用函数时指定axis=0。 由于输入矩阵沿0轴降维以生成输出向量，因此输入轴0的维数在输出形状中消失。

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
沿着行和列对矩阵求和，等价于对矩阵的所有元素进行求和。

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
#### 2.3.6.1 非降维求和

但是，有时在调用函数来计算总和或均值时保持轴数不变会很有用。

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
    <tf.Tensor: shape=(5, 1), dtype=float32, numpy=
    array([[ 6.],
           [22.],
           [38.],
           [54.],
           [70.]], dtype=float32)>
    ```
例如，由于sum_A在对每行进行求和后仍保持两个轴，我们可以通过广播将A除以sum_A。

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
           [0.22857143, 0.24285714, 0.25714287, 0.2714286 ]], dtype=float32)>
    ```
### 2.3.7 点积
我们已经学习了按元素操作、求和及平均值。 另一个最基本的操作之一是点积。给定两个向量，他们的点积是相应元素的乘积的和（**注意：**我们在本书中使用“点积”一词来表示点积）。

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
点积在很多场合都很有用。 例如，给定一组由向量$\mathbf{x} \in \mathbb{R}^d$给出的值，和一组由向量$\mathbf{w} \in \mathbb{R}^d$给出的权重，我们可以通过元素$x_i$和$w_i$的乘积的和来得到一个标量：$\mathbf{x}^\top \mathbf{w} = \sum_{i=1}^d x_i w_i.$ 注意，我们可以将向量$\mathbf{x}$视为是一个行向量，将$\mathbf{w}$视为是一个列向量，其点积为一个矩阵：$\mathbf{x}^\top \mathbf{w} = \mathbf{x}^\top \mathbf{w}.$

### 2.3.8 矩阵-向量积

现在我们知道如何计算点积，我们可以开始理解矩阵-向量积(matrix-vector product)。回顾分别在标量和向量中定义的矩阵$\mathbf{A} \in \mathbb{R}^{m \times n}$和向量$\mathbf{x} \in \mathbb{R}^n$。让我们将矩阵$\mathbf{A}$用它的行向量表示：

$$ \mathbf{A} = \begin{bmatrix} \mathbf{a}^\top_1 \\ \mathbf{a}^\top_2 \\ \vdots \\ \mathbf{a}^\top_m \end{bmatrix}, $$

其中每个$\mathbf{a}^\top_i \in \mathbb{R}^n$都是一个行向量，表示$\mathbf{A}$的第$i$行。矩阵向量积$\mathbf{A}\mathbf{x}$是一个长度为$m$的列向量，其第$i$个元素是点积$\mathbf{a}^\top_i \mathbf{x}$：

$$ \mathbf{A}\mathbf{x} = \begin{bmatrix} \mathbf{a}^\top_1 \\ \mathbf{a}^\top_2 \\ \vdots \\ \mathbf{a}^\top_m \end{bmatrix} \mathbf{x} = \begin{bmatrix} \mathbf{a}^\top_1 \mathbf{x} \\ \mathbf{a}^\top_2 \mathbf{x} \\ \vdots \\ \mathbf{a}^\top_m \mathbf{x} \end{bmatrix}. $$

我们可以吧一个矩阵$\mathbf{A} \in \mathbb{R}^{m \times n}$看作一个从$\mathbb{R}^n$到$\mathbb{R}^m$的转换。这些转换是非常有用的，例如可以用方阵的乘法来表示旋转。后续章节将讲到，我们也可以使用矩阵-向量积来描述在给定前一层的值时， 求解神经网络每一层所需的复杂计算。

=== "PYTORCH"

    ```python
    A.shape, x.shape, torch.mv(A, x)
    ```

    ```text
    (torch.Size([5, 4]), torch.Size([4]), tensor([ 14.,  38.,  62.,  86., 110.]))
    ```
=== "TENSORFLOW"

    ```python
    A.shape, x.shape, tf.linalg.matvec(A, x)
    ```

    ```text
    (<tf.Tensor: shape=(5, 4), dtype=float32, numpy=
    array([[ 0.,  1.,  2.,  3.],
           [ 4.,  5.,  6.,  7.],
           [ 8.,  9., 10., 11.],
           [12., 13., 14., 15.],
           [16., 17., 18., 19.]], dtype=float32)>,
     TensorShape([4]),
     <tf.Tensor: shape=(5,), dtype=float32, numpy=array([ 14.,  38.,  62.,  86., 110.], dtype=float32)>)
    ```
### 2.3.9 矩阵-矩阵乘法

在掌握点积和矩阵-向量积的知识后， 那么**矩阵-矩阵乘法**（matrix-matrix multiplication）应该很简单。

假设我们有两个矩阵$\mathbf{A} \in \mathbb{R}^{n \times k}$和$\mathbf{B} \in \mathbb{R}^{k \times m}$：

$$ \mathbf{A} = \begin{bmatrix} \mathbf{a_{11}} & \mathbf{a_{12}} & \cdots & \mathbf{a_{1k}} \\ \mathbf{a_{21}} & \mathbf{a_{22}} & \cdots & \mathbf{a_{2k}} \\ \vdots & \vdots & \ddots & \vdots \\ \mathbf{a_{n1}} & \mathbf{a_{n2}} & \cdots & \mathbf{a_{nk}} \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} \mathbf{b_{11}} & \mathbf{b_{12}} & \cdots & \mathbf{b_{1m}} \\ \mathbf{b_{21}} & \mathbf{b_{22}} & \cdots & \mathbf{b_{2m}} \\ \vdots & \vdots & \ddots & \vdots \\ \mathbf{b_{k1}} & \mathbf{b_{k2}} & \cdots & \mathbf{b_{km}} \end{bmatrix}. $$

用行向量$\mathbf{a}^\top_{i} \in \mathbb{R}^k$表示矩阵$\mathbf{A}$的第$i$行，用列向量$\mathbf{b}_{j} \in \mathbb{R}^k$表示矩阵$\mathbf{B}$的第$j$列。要生成矩阵积$\mathbf{C} = \mathbf{A}\mathbf{B}$，最简单的方法是考虑$\mathbf{A}$的行向量和$\mathbf{B}$的列向量：

$$ \mathbf{A} = \begin{bmatrix} \mathbf{a}^\top_{1} \\ \mathbf{a}^\top_{2} \\ \vdots \\ \mathbf{a}^\top_{n} \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} \mathbf{b}_{1} & \mathbf{b}_{2} & \cdots & \mathbf{b}_{m} \end{bmatrix}. $$

当我们简单地将每个元素$c_{ij}$计算为点积$\mathbf{a}^\top_i \mathbf{b}_j$时，我们得到了矩阵$\mathbf{C} \in \mathbb{R}^{n \times m}$：

$$ \mathbf{C} = \begin{bmatrix} \mathbf{a}^\top_{1} \\ \mathbf{a}^\top_{2} \\ \vdots \\ \mathbf{a}^\top_{n} \end{bmatrix} \begin{bmatrix} \mathbf{b}_{1} & \mathbf{b}_{2} & \cdots & \mathbf{b}_{m} \end{bmatrix} = \begin{bmatrix} \mathbf{a}^\top_{1} \mathbf{b}_{1} & \mathbf{a}^\top_{1} \mathbf{b}_{2} & \cdots & \mathbf{a}^\top_{1} \mathbf{b}_{m} \\ \mathbf{a}^\top_{2} \mathbf{b}_{1} & \mathbf{a}^\top_{2} \mathbf{b}_{2} & \cdots & \mathbf{a}^\top_{2} \mathbf{b}_{m} \\ \vdots & \vdots & \ddots & \vdots \\ \mathbf{a}^\top_{n} \mathbf{b}_{1} & \mathbf{a}^\top_{n} \mathbf{b}_{2} & \cdots & \mathbf{a}^\top_{n} \mathbf{b}_{m} \end{bmatrix}. $$

我们可以将矩阵-矩阵乘法$\\mathbf{A}\mathbf{B}$看作简单地执行m次矩阵-向量积并将结果拼接在一起，得到一个$n \times m$矩阵。

=== "PYTORCH"

    ```python
    B = torch.ones(4, 3)
    torch.mm(A, B)
    ```

    ```text
    tensor([[ 6.,  6.,  6.],
            [22., 22., 22.],
            [38., 38., 38.],
            [54., 54., 54.],
            [70., 70., 70.]])
    ```
=== "TENSORFLOW"

    ```python
    B = tf.ones((4, 3))
    tf.matmul(A, B)
    ```

    ```text
    <tf.Tensor: shape=(5, 3), dtype=float32, numpy=
    array([[ 6.,  6.,  6.],
           [22., 22., 22.],
           [38., 38., 38.],
           [54., 54., 54.],
           [70., 70., 70.]], dtype=float32)>
    ```

### 2.3.10 范数
线性代数中最有用的一些运算符是范数（norm）。 非正式地说，向量的范数是表示一个向量有多大。 这里考虑的大小（size）概念不涉及维度，而是分量的大小。

在线性代数中，向量范数是将向量映射到标量的函数$f$。 给定任意向量$\mathbf{x}$，向量范数要满足一些属性。第一个性质是：如果我们按常数因子$\alpha$缩放$\mathbf{x}$的所有元素，其范数也会按相同常数因子的绝对值缩放：

$$\| \alpha \mathbf{x} \| = |\alpha| \| \mathbf{x} \|. $$

第二个性质是三角不等式（triangle inequality）。 它表明对于任意两个向量$\mathbf{x}$和$\mathbf{y}$，向量之和的范数不大于向量范数的和：

$$ \| \mathbf{x} + \mathbf{y} \| \leq \| \mathbf{x} \| + \| \mathbf{y} \| . $$

第三个性质简单地说范数必须非负：

$$ \| \mathbf{x} \| \geq 0. $$

这是有道理的，因为在大多数情况下，任何东西的最小大小为零。最后一个性质规定了什么样的向量具有零范数：

$$ \| \mathbf{x} \| = 0 \Leftrightarrow \mathbf{x} = \mathbf{0}. $$

范数听起来很像距离的度量。 欧几里得距离和毕达哥拉斯定理中的非负性概念和三角不等式可能会给出一些启发。 事实上，欧几里得距离是一个$L_2$范数: 假设$n$维向量$\mathbf{x}$的坐标为$x_1, \ldots, x_n$, 其$L_2$范数是向量元素平方和的平方根：

$$ \| \mathbf{x} \|_2 = \sqrt{\sum_{i=1}^n x_i^2}, $$

其中，在$L_2$范数中常常省略下标2，也就是说$\|\mathbf{x}\|$等价于$\|\mathbf{x}\|_2$。 在代码中，我们可以按如下方式计算向量的$L_2$范数。

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
在机器学习中，我们更经常地使用$L_2$范数的平方。 你还会经常遇到$L_1$范数，它表示为向量元素的绝对值之和：

$$ \| \mathbf{x} \|_1 = \sum_{i=1}^n \left|x_i \right|. $$

与$L_2$范数相比，$L_1$范数受异常值的影响较小。为了计算$L_1$范数，我们将绝对值函数和按元素求和组合起来。

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

$L_2$范数和$L_1$范数都是更一般形式$L_p$范数的特例：

$$ \| \mathbf{x} \|_p = \left( \sum_{i=1}^n \left|x_i \right|^p \right)^{\frac{1}{p}}. $$

类似于向量的$L_2$范数，矩阵$\mathbf{X} \in \mathbb{R}^{m \times n}$的**弗罗贝尼乌斯范数**（Frobenius norm）是矩阵元素平方和的平方根：

$$ \|\mathbf{X}\|_F = \sqrt{\sum_{i=1}^m \sum_{j=1}^n x_{ij}^2}. $$

弗罗贝尼乌斯范数满足向量范数的所有性质。 它就像是将矩阵展平成向量后计算其$L_2$范数一样。

=== "PYTORCH"

    ```python
    torch.norm(torch.ones((4, 9)))
    ```

    ```text
    tensor(6.)
    ```
=== "TENSORFLOW"

    ```python
    tf.norm(tf.ones((4, 9)))
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=6.0>
    ```
#### 2.3.10.1 范数和目标

在深度学习中，我们经常试图解决优化问题： 最大化分配给观测数据的概率; 最小化预测和真实观测之间的距离。 用向量表示物品（如单词、产品或新闻文章），以便最小化相似项目之间的距离，最大化不同项目之间的距离。 目标，或许是深度学习算法最重要的组成部分（除了数据），通常被表达为范数。

### 2.3.11 关于线性代数的更多信息¶

仅用一节，我们就教会了阅读本书所需的、用以理解现代深度学习的线性代数。 线性代数还有很多，其中很多数学对于机器学习非常有用。 例如，矩阵可以分解为因子，这些分解可以显示真实世界数据集中的低维结构。 机器学习的整个子领域都侧重于使用矩阵分解及其向高阶张量的泛化，来发现数据集中的结构并解决预测问题。 当开始动手尝试并在真实数据集上应用了有效的机器学习模型，你会更倾向于学习更多数学。 因此，这一节到此结束，本书将在后面介绍更多数学知识。

### 2.3.12 小结

- 标量、向量、矩阵和张量是线性代数中的基本数学对象。

- 向量泛化自标量，矩阵泛化自向量。

- 标量、向量、矩阵和张量分别具有零、一、二和任意数量的轴。

- 一个张量可以通过sum和mean沿指定的轴降低维度。

- 两个矩阵的按元素乘法被称为他们的Hadamard积。它与矩阵乘法不同。

- 在深度学习中，我们经常使用范数，如$L_1$范数、$L_2$范数、范数和Frobenius范数。

- 我们可以对标量、向量、矩阵和张量执行各种操作。


## 2.4 微积分

在2500年前，古希腊人把一个多边形分成三角形，并把它们的面积相加，才找到计算多边形面积的方法。 为了求出曲线形状（比如圆）的面积，古希腊人在这样的形状上刻内接多边形。 内接多边形的等长边越多，就越接近圆。 这个过程也被称为逼近法（method of exhaustion）。事实上，逼近法就是积分（integral calculus）的起源。 2000多年后，微积分的另一支，微分（differential calculus）被发明出来。 在微分学最重要的应用是优化问题，即考虑如何把事情做到最好。 正如在 2.3.10.1节中讨论的那样， 这种问题在深度学习中是无处不在的。

在深度学习中，我们“训练”模型，不断更新它们，使它们在看到越来越多的数据时变得越来越好。 通常情况下，变得更好意味着最小化一个损失函数（loss function）， 即一个衡量“模型有多糟糕”这个问题的分数。 最终，我们真正关心的是生成一个模型，它能够在从未见过的数据上表现良好。 但“训练”模型只能将模型与我们实际能看到的数据相拟合。 因此，我们可以将拟合模型的任务分解为两个关键问题：

- 优化（optimization）：用模型拟合观测数据的过程；
- 泛化（generalization）：数学原理和实践者的智慧，能够指导我们生成出有效性超出用于训练的数据集本身的模型。

### 2.4.1 导数

我们首先讨论导数的计算，这是几乎所有深度学习优化算法的关键步骤。 在深度学习中，我们通常选择对于模型参数可微的损失函数。 简而言之，对于每个参数， 如果我们把这个参数增加或减少一个无穷小的量，可以知道损失会以多快的速度增加或减少。

假设我们有一个函数$f: \mathbb{R} \rightarrow \mathbb{R}$，它的输入和输出都是标量。如果函数$f$的导数存在，这个极限被定义为：

$$ f'(x) = \lim_{h \rightarrow 0} \frac{f(x+h) - f(x)}{h}.$$

如果导数$f'(a)$存在，则函数$f$在点$a$是可微的。如果$f$在区间$[a, b]$上的每个点都可微，那么$f$在区间$[a, b]$上是可微的。如果$f$在区间$(-\infty, \infty)$上是可微的，则$f$是可微的。函数$f$的导数$f'$也是一个函数：它将$x$映射到导数$f'(x)$。同样，如果$f'$在某个区间上是可微的，我们可以计算它的导数，它被称为$f$的二阶导数$f''$，以此类推。

我们可以将导数$f'(x)$解释为：当$x$在$f(x)$处增加$h$时，$f(x)$相对于$h$的变化率。因此，当$h$接近0时，$f(x)$相对于$h$的变化率是$f'(x)$。我们也可以将导数解释为：$f(x)$的瞬时变化率。例如，如果$f(x)$表示一个人的位置随时间的变化，那么$f'(x)$就是这个人的瞬时速度。如果$f(x)$表示一个人的速度随时间的变化，那么$f'(x)$就是这个人的瞬时加速度。

### 2.4.2 偏导数

到目前为止，我们只讨论了仅含一个变量的函数的微分。 在深度学习中，函数通常依赖于许多变量。 因此，我们需要将微分的思想推广到多元函数（multivariate function）上。

设$y = f(x_1, x_2, \ldots, x_n)$是一个具有$n$个变量的函数。 $f$的偏导数$\frac{\partial y}{\partial x_i}$（partial derivative）衡量了$f$相对于变量$x_i$的变化率，而将其他变量$x_j (j \neq i, j = 1, 2, \ldots, n)$视为常数。 要计算$\frac{\partial y}{\partial x_i}$，我们可以将$f(x_1, x_2, \ldots, x_n)$视为一个关于一个变量$x_i$的函数，而将其他变量$x_j (j \neq i, j = 1, 2, \ldots, n)$视为常数。 然后，我们计算这个函数的导数，就像我们在单变量函数的情况下所做的那样，而将其他变量视为常数。

### 2.4.3 梯度

我们可以连结一个多元函数对其所有变量的偏导数，以得到该函数的梯度（gradient）向量。 具体而言，设函数$f: \mathbb{R}^n \rightarrow \mathbb{R}$的输入是一个$n$维向量$\mathbf{x} = [x_1, x_2, \ldots, x_n]^\top$，它的输出是标量。 $f(\mathbf{x})$相对于$\mathbf{x}$的梯度是一个包含$n$个偏导数的向量：

$$ \nabla_{\mathbf{x}} f(\mathbf{x}) = \begin{bmatrix} \frac{\partial f(\mathbf{x})}{\partial x_1} \\ \frac{\partial f(\mathbf{x})}{\partial x_2} \\ \vdots \\ \frac{\partial f(\mathbf{x})}{\partial x_n} \end{bmatrix}. $$

其中 $\nabla_{\mathbf{x}} f(\mathbf{x})$ 读作“$f(\mathbf{x})$关于$\mathbf{x}$的梯度”。 在本书中，我们使用梯度和导数这两个术语。 梯度是偏导数的向量，而导数是偏导数的标量。

假设$x$为$n$维向量，在微分多元函数时，经常使用以下规则：

1. 对于所有的$\mathbf{A} \in \mathbb{R}^{m \times n}$，都有$\nabla_{\mathbf{x}} \mathbf{A}\mathbf{x} = \mathbf{A}^\top$。

2. 对于所有的$\mathbf{A} \in \mathbb{R}^{m \times n}$，都有$\nabla_{\mathbf{x}} \mathbf{x}^\top\mathbf{A} = \mathbf{A}$。

3. 对于所有的$\mathbf{A} \in \mathbb{R}^{m \times n}$，都有$\nabla_{\mathbf{x}} \mathbf{x}^\top\mathbf{A}\mathbf{x} = (\mathbf{A} + \mathbf{A}^\top)\mathbf{x}$。

4. 对于所有的可微的单变量函数$f$，都有$\nabla_{\mathbf{x}} f(\mathbf{x}) = \left[\frac{\partial f(\mathbf{x})}{\partial \mathbf{x}}\right]^\top$。

### 2.4.4 链式法则

然而，上面方法可能很难找到梯度。 这是因为在深度学习中，多元函数通常是复合（composite）的， 所以难以应用上述任何规则来微分这些函数。 幸运的是，链式法则可以被用来微分复合函数。

假设$y=f(u)$和$u=g(x)$是两个函数，其中$u$是$x$的函数，$y$是$u$的函数。 为了找到复合函数$y=f(g(x))$关于$x$的导数，链式法则给出了以下关系：

$$ \frac{dy}{dx} = \frac{dy}{du} \frac{du}{dx}. $$

现在考虑一个更一般的场景，即函数具有任意数量的变量的情况。 假设可微分函数$y$是$x_1, x_2, \ldots, x_n$的函数，其中$x_i$是$t_1, t_2, \ldots, t_m$的函数。 为了计算导数$\frac{dy}{dt_i}$，我们可以使用链式法则：

$$ \frac{dy}{dt_i} = \frac{dy}{dx_1} \frac{dx_1}{dt_i} + \frac{dy}{dx_2} \frac{dx_2}{dt_i} + \cdots + \frac{dy}{dx_n} \frac{dx_n}{dt_i}. $$

### 2.4.5 小结

- 微分和积分是微积分的两个分支，前者可以应用于深度学习中的优化问题。

- 导数可以被解释为函数相对于其变量的瞬时变化率，它也是函数曲线的切线的斜率。

- 梯度是一个向量，其分量是多变量函数相对于其所有变量的偏导数。

- 链式法则可以用来微分复合函数。

## 2.5 自动微分
求导是几乎所有深度学习优化算法的关键步骤。 虽然求导的计算很简单，只需要一些基本的微积分。 但对于复杂的模型，手工进行更新是一件很痛苦的事情（而且经常容易出错）。

深度学习框架通过自动计算导数，即自动微分（automatic differentiation）来加快求导。 实际中，根据设计好的模型，系统会构建一个**计算图**（computational graph）， 来跟踪计算是**哪些数据通过哪些操作组合起来产生输出**。 自动微分使系统能够随后反向传播梯度。 这里，**反向传播**（backpropagate）意味着跟踪整个计算图，填充关于每个参数的偏导数。

### 2.5.1 一个简单的例子

作为一个演示例子，假设我们想对函数$y = 2\mathbf{x}^\top\mathbf{x}$关于列向量$\mathbf{x}$求导，其中$\mathbf{x} \in \mathbb{R}^d$。 首先，我们创建变量`x`并为其分配一个初始值。

=== "PYTORCH"

    ```python
    import torch

    x = torch.arange(4.0)
    x
    ```

    ```text
    tensor([0., 1., 2., 3.])
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf

    x = tf.range(4, dtype=tf.float32)
    x
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([0., 1., 2., 3.], dtype=float32)>
    ```

在我们计算`y`关于`x`的梯度之前，我们需要一个地方来存储梯度。重要的是，我们不会在每次对一个参数求导时都分配新的内存。 因为我们经常会成千上万次地更新相同的参数，每次都分配新的内存可能很快就会将内存耗尽。 注意，一个标量函数关于向量$x$的梯度是向量，并且与$x$具有相同的形状。

=== "PYTORCH"

    ```python
    x.requires_grad_(True)
    ```

    ```text
    tensor([0., 1., 2., 3.], requires_grad=True)
    ```
=== "TENSORFLOW"

    ```python
    x = tf.Variable(x)
    ```

    ```text

现在计算$y$

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
x是一个长度为4的向量，计算`x`和`x`的内积，得到了我们赋值给`y`的标量输出。接下来，我们可以通过调用反向传播函数来自动计算`y`关于`x`每个分量的梯度，然后打印这些梯度。

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
    x_grad = t.gradient(y, x)
    x_grad
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([ 0.,  4.,  8., 12.], dtype=float32)>
    ```
函数$y = 2\mathbf{x}^\top\mathbf{x}$关于$\mathbf{x}$的梯度应为$4\mathbf{x}$。 让我们快速验证我们想到的结果是否正确。

=== "PYTORCH"

    ```python
    x.grad == 4 * x
    ```

    ```text
    tensor([True, True, True, True])
    ```
=== "TENSORFLOW"

    ```python
    x_grad == 4 * x
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=bool, numpy=array([ True,  True,  True,  True])>
    ```
现在计算`x`的另一个函数。

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
    t.gradient(y, x)  # 被新计算的梯度覆盖
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([1., 1., 1., 1.], dtype=float32)>
    ```

### 2.5.2 非标量变量的反向传播

当y不是标量时，向量y关于向量x的导数的最自然解释是一个矩阵。 对于高阶和高维的y和x，求导的结果可以是一个高阶张量。

然而，虽然这些更奇特的对象确实出现在高级机器学习中（包括深度学习中）， 但当调用向量的反向计算时，我们通常会试图计算一批训练样本中每个组成部分的损失函数的导数。 这里，我们的目的不是计算微分矩阵，而是单独计算批量中每个样本的偏导数之和。

=== "PYTORCH"

    ```python
    # 对非标量调用backward需要传入一个gradient参数，该参数指定微分函数关于self的梯度。
    # 本例只想求偏导数的和，所以传递一个1的梯度是合适的
    x.grad.zero_()
    y = x * x
    # 等价于y.backward(torch.ones(len(x)))
    y.sum().backward()
    x.grad
    ```

    ```text
    tensor([0., 2., 4., 6.])
    ```
=== "TENSORFLOW"

    ```python
    with tf.GradientTape() as t:
        y = x * x
    t.gradient(y, x)# 等价于y=tf.reduce_sum(x*x)
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=float32, numpy=array([0., 2., 4., 6.], dtype=float32)>
    ```

### 2.5.3 分离计算

有时，我们希望将某些计算移动到记录的计算图之外。 例如，假设y是作为x的函数计算的，而z则是作为y和x的函数计算的。 想象一下，我们想计算z关于x的梯度，但由于某种原因，希望将y视为一个常数， 并且只考虑到x在y被计算后发挥的作用。

这里可以分离y来返回一个新变量u，该变量与y具有相同的值， 但丢弃计算图中如何计算y的任何信息。 换句话说，梯度不会向后流经u到x。 因此，下面的反向传播函数计算z=u*x关于x的偏导数，同时将u作为常数处理， 而不是z=x*x*x关于x的偏导数。

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
    with tf.GradientTape() as t:
        y = x * x
        u = tf.stop_gradient(y)
        z = u * x
    t.gradient(z, x) == u
    ```

    ```text
    <tf.Tensor: shape=(4,), dtype=bool, numpy=array([ True,  True,  True,  True])>
    ```

### 2.5.4 Python控制流的梯度计算

使用自动微分的一个好处是： 即使构建函数的计算图需要通过Python控制流（例如，条件、循环或任意函数调用），我们仍然可以计算得到的变量的梯度。 在下面的代码中，while循环的迭代次数和if语句的结果都取决于输入a的值。

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

让我们计算梯度

=== "PYTORCH"
    ```python
    a = torch.randn(size=(), requires_grad=True)
    d = f(a)
    d.backward()
    ```

=== "TENSORFLOW"
    ```python
    a = tf.Variable(tf.random.normal(shape=()))
    with tf.GradientTape() as t:
        d = f(a)
    d_grad = t.gradient(d, a)
    d_grad
    ```

    ```text
    <tf.Tensor: shape=(), dtype=float32, numpy=51200.0>
    ```
我们现在可以分析上面定义的f函数。 请注意，它在其输入a中是分段线性的。 换言之，对于任何a，存在某个常量标量k，使得f(a)=k*a，其中k的值取决于输入a，因此可以用d/a验证梯度是否正确。

=== "PYTORCH"
    ```python
    a.grad == d / a
    ```

    ```text
    tensor(True)
    ```
=== "TENSORFLOW"
    ```python
    d_grad == d / a
    ```

    ```text
    <tf.Tensor: shape=(), dtype=bool, numpy=True>
    ```
### 2.5.5 小结
深度学习框架可以自动计算导数：我们首先将梯度附加到想要对其计算偏导数的变量上，然后记录目标值的计算，执行它的反向传播函数，并访问得到的梯度。


## 2.6 概率与统计
简单地说，机器学习就是做出预测。

根据病人的临床病史，我们可能想预测他们在下一年心脏病发作的概率。 在飞机喷气发动机的异常检测中，我们想要评估一组发动机读数为正常运行情况的概率有多大。 在强化学习中，我们希望智能体（agent）能在一个环境中智能地行动。 这意味着我们需要考虑在每种可行的行为下获得高奖励的概率。 当我们建立推荐系统时，我们也需要考虑概率。 例如，假设我们为一家大型在线书店工作，我们可能希望估计某些用户购买特定图书的概率。 为此，我们需要使用概率学。 有完整的课程、专业、论文、职业、甚至院系，都致力于概率学的工作。 所以很自然地，我们在这部分的目标不是教授整个科目。 相反，我们希望教给读者基础的概率知识，使读者能够开始构建第一个深度学习模型， 以便读者可以开始自己探索它。

### 2.6.1 基本概率论

在统计学中，我们把从概率分布中抽取样本的过程称为抽样（sampling）。 笼统来说，可以把分布（distribution）看作对事件的概率分配， 稍后我们将给出的更正式定义。 将概率分配给一些离散选择的分布称为多项分布（multinomial distribution）。

#### 2.6.1.1 概率论公理

#### 2.6.1.2 随机变量

### 2.6.2 处理多个随机变量

#### 2.6.2.1 联合概率

#### 2.6.2.2 条件概率

#### 2.6.2.3 贝叶斯定理
使用条件概率的定义，我们可以得出统计学中最有用的方程之一： Bayes定理（Bayes’ theorem）。 
根据**乘法法则**（multiplication rule ）可得到 $P(A, B) =  P(B \mid A) P(A)$。
根据**对称性**，$P(A, B) = P(A \mid B) P(B)$。 因此，$P(A \mid B) P(B) = P(B \mid A) P(A)$。 
通过重新排列，我们得到**贝叶斯定理**：
$$
P(A \mid B) = \frac{P(B \mid A) P(A)}{P(B)}
$$

#### 2.6.2.4 边际化 （边际概率，边际分布）

**求和法则**

#### 2.6.2.5 独立性
另一个有用属性是依赖（dependence）与独立（independence）。 如果两个随机变量A
和B是独立的，意味着事件A的发生跟B事件的发生无关。

#### 2.6.2.6 应用

### 2.6.3 期望和方差

### 2.6.4 小结

- 我们可以从概率分布中采样。

- 我们可以使用联合分布、条件分布、Bayes定理、边缘化和独立性假设来分析多个随机变量。

- 期望和方差为概率分布的关键特征的概括提供了实用的度量形式。

## 2.7 文档
由于篇幅限制，本书不可能介绍每一个PyTorch函数和类。 API文档、其他教程和示例提供了本书之外的大量文档。 本节提供了一些查看PyTorch API的指导。

### 2.7.1 查找模块中的所有函数和类

要查找模块中的所有函数和类，我们可以使用`dir`函数。 例如，我们可以查询随机数生成模块中的所有属性：

=== "PYTORCH"

    ```python
    import torch
    ```

    ```python
    dir(torch.distributions)
    ```

    ```text
    ['Bernoulli',
    'Beta',
    'Binomial',
    'Categorical',
    'Cauchy',
    'Chi2',
    'Dirichlet',
    'Distribution',
    'Exponential',
    'ExponentialFamily',
    'FisherSnedecor',
    'Gamma',
    'Geometric',
    'Gumbel',
    'HalfCauchy',
    'HalfNormal',
    'Independent',
    'KLDivLoss',
    'LKJCholesky',
    'Laplace',
    'LowRankMultivariateNormal',
    'MixtureSameFamily',
    'Multinomial',
    'MultivariateNormal',
    'NegativeBinomial',
    'Normal',
    'OneHotCategorical',
    'Pareto',
    'Poisson',
    'RelaxedBernoulli',
    'RelaxedOneHotCategorical',
    'StudentT',
    'TransformedDistribution',
    'Uniform',
    'VonMises',
    'Weibull',
    'Zipf',
    '__builtins__',
    '__cached__',
    '__doc__',
    '__file__',
    '__loader__',
    '__name__',
    '__package__',
    '__path__',
    '__spec__',
    '_lazy_init',
    '_standard_gamma',
    'constraints',
    'functional',
    'kl_divergence',
    'lazy_property',
    'register_kl',
    'transform_to',
    'utils']
    ```
=== "TENSORFLOW"

    ```python
    import tensorflow as tf
    ```

    ```python
    dir(tf.distributions)
    ```

    ```text
        ['Bernoulli',
    'Beta',
    'Binomial',
    'Categorical',
    'Cauchy',
    'Chi2',
    'Dirichlet',
    'Distribution',
    'Exponential',
    'ExponentialFamily',
    'FisherSnedecor',
    'Gamma',
    'Geometric',
    'Gumbel',
    'HalfCauchy',
    'HalfNormal',
    'Independent',
    'KLDivLoss',
    'LKJCholesky',
    'Laplace',
    'LowRankMultivariateNormal',
    'MixtureSameFamily',
    'Multinomial',
    'MultivariateNormal',
    'NegativeBinomial',
    'Normal',
    'OneHotCategorical',
    'Pareto',
    'Poisson',
    'RelaxedBernoulli',
    'RelaxedOneHotCategorical',
    'StudentT',
    'TransformedDistribution',
    'Uniform',
    'VonMises',
    'Weibull',
    'Zipf',
    '__builtins__',
    '__cached__',
    '__doc__',
    '__file__',
    '__loader__',
    '__name__',
    '__package__',
    '__path__',
    '__spec__',
    '_lazy_init',
    '_standard_gamma',
    'constraints',
    'functional',
    'kl_divergence',
    'lazy_property',
    'register_kl',
    'transform_to',
    'utils']
    ```

通常可以忽略以“__”（双下划线）开始和结束的函数，它们是Python中的特殊对象， 或以单个“_”（单下划线）开始的函数，它们通常是内部函数。 根据剩余的函数名或属性名，我们可能会猜测这个模块提供了各种生成随机数的方法， 包括从均匀分布（uniform）、正态分布（normal）和多项分布（multinomial）中采样。

### 2.7.2 查找特定函数和类的所有成员

我们可以使用`help`函数来查找特定函数或类的所有成员。 让我们从`torch.randn`函数开始。 如果我们不知道如何使用它，我们可以调用它并使用`help`函数来查找用法。

=== "PYTORCH"

    ```python
    help(torch.randn)
    ```

    ```text
    Help on built-in function randn:

    randn(...)
        randn(*size, out=None, dtype=None, layout=torch.strided, device=None, requires_grad=False) -> Tensor

        Returns a tensor filled with random numbers from a normal distribution
        with mean `0` and variance `1` (also called the standard normal
        distribution).

        .. math::
            \text{{out}}_{{i}} \sim \mathcal{{N}}(0, 1)

        The shape of the tensor is defined by the variable argument `size`.

        Args:
            {input}

        Example::

            >>> torch.randn(4)
            tensor([-2.1436,  0.9966,  0.6479, -0.4219])
            >>> torch.randn(2, 3)
            tensor([[ 1.2074, -0.9477, -0.1569],
                    [ 0.1886, -0.3753, -0.1837]])
        """
        return torch._C._VariableFunctions.randn(size, dtype=dtype, layout=layout, device=device, requires_grad=requires_grad)
    ```
=== "TENSORFLOW"

    ```python
    help(tf.random.normal)
    ```

    ```text
    Help on built-in function randn:

    randn(...)
        randn(*size, out=None, dtype=None, layout=torch.strided, device=None, requires_grad=False) -> Tensor

        Returns a tensor filled with random numbers from a normal distribution
        with mean `0` and variance `1` (also called the standard normal
        distribution).

        .. math::
            \text{{out}}_{{i}} \sim \mathcal{{N}}(0, 1)

        The shape of the tensor is defined by the variable argument `size`.

        Args:
            {input}

        Example::

            >>> torch.randn(4)
            tensor([-2.1436,  0.9966,  0.6479, -0.4219])
            >>> torch.randn(2, 3)
            tensor([[ 1.2074, -0.9477, -0.1569],
                    [ 0.1886, -0.3753, -0.1837]])
        """
        return torch._C._VariableFunctions.randn(size, dtype=dtype, layout=layout, device=device, requires_grad=requires_grad)
    ```
### 2.7.3 小结

- 官方文档提供了本书之外的大量描述和示例。

- 可以通过调用dir和help函数或在Jupyter记事本中使用?和??查看API的用法文档。