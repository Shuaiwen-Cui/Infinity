# C 语言学习笔记

## 目录

CH1 - 导言
CH2 - 类型、运算符与表达式
CH3 - 控制流
CH4 - 函数与程序结构
CH5 - 指针与数组
CH6 - 结构
CH7 - 输入与输出
CH8 - UNIX 系统接口

## CH1 - 导言

一些简单案例。

## CH2 - 类型、运算符与表达式
- **变量**和**常量**是程序处理的两种基本数据对象。
- **声明语句**说明变量的名字及类型，也可以指定变量的初值。
- **运算符**指定将要进行的操作。
- **表达式**则把变量与常量组合起来生成新的值。
- 对象的**类型**决定该对象可取值的集合以及可以对该对象执行的操作。

### 2.1 变量名
对变量的命名与符号常量的命名存在一些限制条件：
- 名字是由字母和数字组成的序列，但其第一个字符必须为字母
  - 下划线“_”被看做是字母，通常用于命名较长的变量名，以提高其可读性
  - 由于例程的名字通常以下划线开头，因此变量名不要以下划线开头
- 大写字母与小写字母是有区别的
  - 通常，变量名用小写字母，符号常量用大写字母
- 选择的变量名要能够尽量从字面上表达变量的用途，这样做不容易引起混淆。
  - 局部变量一般使用较短的变量名（尤其是循环控制变量）
  - 外部变量使用较长的名字

### 2.2 数据类型及长度
C 语言只提供了下列几种基本数据类型：

- char 字符型，一个字节, 可以存放本地字符集中的一个字符
- int 整型，通常反映了所用机器中整数的最自然长度 
- float 单精度浮点型
- double 双精度浮点型

此外，还可以在这些基本数据类型的前面加上一些限定符。short 与 long 两个限定符用于限定整型：

short int sh;
long int counter;

在上述这种类型的声明中，关键字 int 可以省略。通常很多人也习惯这么做。

short 与 long 两个限定符的引入可以为我们提供满足实际需要的不同长度的整型数。int 通常代表特定机器中整数的自然长度。short 类型通常为 16 位，1ong 类型通常为 32位，int 类型可以为 16 位或 32 位。各编译器可以根据硬件特性自主选择合适的类型长度，但要遵循下列限制：short 与 int 类型至少为 16 位，而 long 类型至少为 32 位，并且 short类型不得长于 int 类型，而 int 类型不得长于 long 类型。

类型限定符 signed 与 unsigned 可用于限定 char 类型或任何整型。unsigned 类型的数总是正值或 0，并遵守算术模 2^n定律，其中 n 是该类型占用的位数。例如，如果 char 对象占用 8 位，那么 unsigned char 类型变量的取值范围为 0～255，而 signed char 类型变量的取值范围则为-128～127（在采用对二的补码的机器上）。不带限定符的 char 类型对象是否带符号则取决于具体机器，但可打印字符总是正值。

### 2.3 常量

#### 整数常量

整数常量属于 int 类型。long 类型的常量以字母 l 或 L 结尾，如123456789L。如果一个整数太大以至于无法用 int 类型表示时，也将被当作 long 类型处理。无符号常量以字母 u 或 U 结尾。后缀 ul 或 UL 表明是 unsigned long 类型。

整型数除了用十进制表示外，还可以用八进制或十六进制表示。带前缀 0 的整型常量表示它为八进制形式；前缀为 0x 或 0X，则表示它为十六进制形式。

#### 浮点数常量

浮点数常量中包含一个小数点（如 123.4）或一个指数（如 1e-2），也可以两者都有。没有后缀的浮点数常量为 double 类型。后缀 f 或 F 表示 float 类型，而后缀 l 或 L 则表示 long double 类型。

#### 字符常量

一个字符常量是一个整数，书写时将一个字符括在单引号中。如果用字符'0'代替这个与具体字符集有关的值（比如 48），那么，程序就无需关心该字符对应的具体值，增加了程序的易读性。字符常量一般用来与其它字符进行比较，但也可以像其它整数一样参与数值运算。

!!! info "转义字符"
    ANSI C 语言中的全部转义字符序列如下所示：

    \a 响铃符       \\\\ 反斜杠

    \b 回退符       \? 问号

    \f 换页符       \' 单引号

    \n 换行符       \" 双引号
    
    \r 回车符       \ooo 八进制数

    \t 横向制表符   \xhh 十六进制数

    \v 纵向制表符

字符常量'\0'表示值为 0 的字符，也就是空字符（null）。我们通常用'\0'的形式代替 0，以强调某些表达式的字符属性，但其数字值为 0。

常量表达式是仅仅只包含常量的表达式。这种表达式在编译时求值，而不在运行时求值。它可以出现在常量可以出现的任何位置。

#### 字符串常量

字符串常量或字符串字面量以双引号括起来的字符序列表示。字符串常量是一个字符数组。字符串的内部表示在末尾有一个空字符'\0'，因此所需的物理存储比引号之间写入的字符数多一个。这种表示也解释了为什么字符串的长度没有限制；程序只需要扫描内存，直到找到'\0'来确定长度。标准库函数strlen(s)返回其字符串参数的长度，不包括'\0'。

我们应该搞清楚字符常量与仅包含一个字符的字符串之间的区别：'x'与"x"是不同的。前者是一个整数，其值是字母 x 在机器字符集中对应的数值（内部表示值）；后者是一个包含一个字符（即字母 x）以及一个结束符'\0'的字符数组。

#### 枚举常量

枚举常量是一种特殊的整型常量。枚举常量是由用户定义的标识符的列表。关键字 enum 说明符用于定义枚举类型，其形式为：

enum 枚举名 {枚举常量表}；

枚举常量表中的第一个枚举常量的默认值为 0，后续枚举常量的值依次加 1。枚举常量的值可以在定义时显式地赋值。如果没有显式地赋值，那么枚举常量的值将比前一个枚举常量的值大 1。枚举常量的值必须是整型常量，且不能重复。

枚举常量的作用域与其它变量的作用域相同，即从定义处到该枚举常量所在的块的结束处。枚举常量的类型为 int。

### 2.4 声明

所有变量都必须先声明后使用，尽管某些变量可以通过上下文隐式地声明。一个声明指定一种变量类型，后面所带的变量表可以包含一个或多个该类型的变量。

一个声明语句中的多个变量可以拆开在多个声明语句中声明。按照这种形式书写代码需要占用较多的空间，但便于向各声明语句中添加注释，也便于以后修改。

还可以在声明的同时对变量进行初始化。在声明中，如果变量名的后面紧跟一个等号以及一个表达式，该表达式就充当对变量进行初始化的初始化表达式。

如果变量不是自动变量，则只能进行一次初始化操作，从概念上讲，应该是在程序开始执行之前进行，并且初始化表达式必须为常量表达式。每次进入函数或程序块时，显式初始化的自动变量都将被初始化一次，其初始化表达式可以是任何表达式。默认情况下，外部变量与静态变量将被初始化为 0。未经显式初始化的自动变量的值为未定义值（即无效值）。

任何变量的声明都可以使用 const 限定符限定。该限定符指定变量的值不能被修改。对数组而言，const 限定符指定数组所有元素的值都不能被修改。const 限定符的使用可以提高程序的可读性，也可以帮助编译器检查程序中的错误。

### 2.5 算术运算符
二元算术运算符包括：+、-、*、/、%（取模运算符）。整数除法会截断结果中的小数部分。表达式 x % y 的结果是 x 除以 y 的余数，当 x 能被 y 整除时，其值为 0。

取模运算符%不能应用于 float 或 double 类型。在有负操作数的情况下，整数除法截取的方向以及取模运算结果的符号取决于具体机器的实现，这和处理上溢或下溢的情况是一样的。

二元运算符+和-具有相同的优先级，它们的优先级比运算符*、/和%的优先级低，而运算符*、/和%的优先级又比一元运算符+和-的优先级低。算术运算符采用从左到右的结合规则。

### 2.6 关系运算符与逻辑运算符
关系运算符包括下列几个运算符：
\> >= < <= 
它们具有相同的优先级。优先级仅次于它们的是相等性运算符：
== != 
关系运算符的优先级比算术运算符低。

逻辑运算符&&与||有一些较为特殊的属性，由&&与||连接的表达式按从左到右的顺序进行求值，并且，在知道结果值为真或假后立即停止计算。绝大多数 C 语言程序运用了这些属性。

根据定义，在关系表达式或逻辑表达式中，如果关系为真，则表达式的结果值为数值 1；如果为假，则结果值为数值 0。

### 2.7 类型转换
当一个运算符的几个操作数类型不同时，就需要通过一些规则把它们转换为某种共同的类型。一般来说，自动转换是指把“比较窄的”操作数转换为“比较宽的”操作数，并且不丢失信息的转换。不允许使用无意义的表达式，例如，不允许把 float 类型的表达式作为下标。针对可能导致信息丢失的表达式，编译器可能会给出警告信息，比如把较长的整型值赋给较短的整型变量，把浮点型值赋值给整型变量，等等，但这些表达式并不非法。

由于 char 类型就是较小的整型，因此在算术表达式中可以自由使用 char 类型的变量，这就为实现某些字符转换提供了很大的灵活性，比如，下面的函数 atoi 就是一例，它将一串数字转换为相应的数值。

将字符类型转换为整型时，我们需要注意一点。C 语言没有指定 char 类型的变量是无符号变量（signed）还是带符号变量（unsigned）。当把一个 char 类型的值转换为 int 类型的值时，其结果有没有可能为负整数？对于不同的机器，其结果也不同，这反映了不同机器结构之间的区别。在某些机器中，如果 char 类型值的最左一位为 1，则转换为负整数（进行“符号扩展”）。而在另一些机器中，把 char 类型值转换为 int 类型时，在 char 类型值的左边添加 0，这样导致的转换结果值总是正值。

C 语言的定义保证了机器的标准打印字符集中的字符不会是负值，因此，在表达式中这些字符总是正值。但是，存储在字符变量中的位模式在某些机器中可能是负的，而在另一些机器上可能是正的。为了保证程序的可移植性，如果要在 char 类型的变量中存储非字符数据，最好指定 signed 或 unsigned 限定符。

C 语言中，很多情况下会进行**隐式的算术类型转换**。一般来说，如果二元运算符（具有两个操作数的运算符称为二元运算符，比如+或*）的两个操作数具有不同的类型，那么在进行运算之前先要把“较低”的类型提升为“较高”的类型，运算的结果为较高的类型。

赋值时也要进行类型转换。赋值运算符右边的值需要转换为左边变量的类型，左边变量的类型即赋值表达式结果的类型。
前面提到过，无论是否进行符号扩展，字符型变量都将被转换为整型变量。当把较长的整数转换为较短的整数或 char 类型时，超出的高位部分将被丢弃。

在任何表达式中都可以使用一个称为强制类型转换的一元运算符强制进行显式类型转换。在下列语句中，表达式将按照上述转换规则被转换为类型名指定的类型：

(类型名) 表达式

### 2.8 自增与自减运算符
C 语言提供了两个用于变量递增与递减的特殊运算符。自增运算符++使其操作数递增 1，自减运算符使其操作数递减 1。

++与--这两个运算符特殊的地方主要表现在：它们既可以用作前缀运算符（用在变量前面，如++n）。也可以用作后缀运算符（用在变量后面，如 n++）。在这两种情况下，其效果都是将变量 n 的值加 1。但是，它们之间有一点不同。表达式++n 先将 n 的值递增 1，然后再使用变量 n 的值，而表达式 n++则是先使用变量 n 的值，然后再将 n 的值递增 1。

### 2.9 位运算符
C 语言提供了 6 个位操作运算符。这些运算符只能作用于整型操作数，即只能作用于带符号或无符号 char、short、int、long 类型：

- & 按位与（AND）
- | 按位或（OR）
- ^ 按位异或（XOR）
- << 左移
- >> 右移
- ~ 按位求反（一元运算符）

按位与运算符&经常用于屏蔽某些二进制位，例如：n = n & 0177;
该语句将 n 中除 7 个低二进制位外的其它各位均置为 0。

按位或运算符|常用于将某些二进制位置为 1，例如：x = x | SET_ON；
该语句将 x 中对应于 SET_ON 中为 1 的那些二进制位置为 1。

按位异或运算符^当两个操作数的对应位不相同时将该位设置为 1，否则，将该位设置为0。 
我们必须将位运算符&、|同逻辑运算符&&、||区分开来，后者用于从左至右求表达式的真值。例如，如果 x 的值为 1，Y 的值为 2，那么，x & y 的结果为 0，而 x && y 的值为 1

移位运算符<<与>>分别用于将运算的左操作数左移与右移，移动的位数则由右操作数指
定（右操作数的值必须是非负值）。因此，表达式 x << 2 将把 x 的值左移 2 位，右边空出的
2 位用 0 填补，该表达式等价于对左操作数乘以 4。

一元运算符~用于求整数的二进制反码，即分别将操作数各二进制位上的 1 变为 0，0 变为 1。

### 2.10 赋值运算符与表达式
大多数二元运算符（即有左、右两个操作数的运算符，比如+）都有一个相应的赋值运算符 op=，其中，op 可以是下面这些运算符之一：

`+ - * / % << >> & ^ |`

如果 expr1 和 expr2 是表达式，那么
`expr1 op= expr2` 

等价于：
`expr1 = (expr1) op (expr2)`


### 2.11 条件表达式

条件表达式 if-else 的一般形式为：

```c

if (表达式 1)
    语句 1
else if (表达式 2)
    语句 2
else if (表达式 3)
    语句 3
```

条件表达式形式（三元）：

`expr1 ? expr2 : expr3`

### 2.12 运算符优先级与求值次序

运算符优先级与求值次序表：

| 运算符 | 优先级 |
| :----: | :----: |
| () [] -> . | 从左到右 |
| ! ~ ++ -- + - * & (类型) sizeof | 从右到左 |
| * / % | 从左到右 |
| + - | 从左到右 |
| << >> | 从左到右 |
| < <= > >= | 从左到右 |
| == != | 从左到右 |
| & | 从左到右 |
| ^ | 从左到右 |
| \| | 从左到右 |
| && | 从左到右 |
| \|\| | 从左到右 |
| ?: | 从右到左 |
| = += -= *= /= %= &= ^= \|= <<= >>= | 从右到左 |
| , | 从左到右 |
