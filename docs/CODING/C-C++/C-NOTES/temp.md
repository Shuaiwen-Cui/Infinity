## CH3 - 控制流
程序语言中的控制流语句用于控制各计算操作执行的次序。

### 3.1 语句与程序块

#### 语句
在 x = 0、i++或 printf(...)这样的表达式之后加上一个分号（;），它们就变成了
语句。在 C 语言中，分号是语句结束符，而 Pascal 等语言却把分号用作语句之间的分隔符。

#### 程序块
用一对花括号“{”与“}”把一组声明和语句括在一起就构成了一个复合语句（也叫作程序块），复合语句在语法上等价于单条语句。右花括号用于结束程序块，其后不需要分号。

### 3.2 if-else 语句

。。。k
if {表达式} 
  语句 1 
else 
  语句 2
。。。

其中 else 部分是可选的。该语句执行时，先计算表达式的值，如果其值为真（即表达式的值
为非 0），则执行语句 1；如果其值为假（即表达式的值为 0），并且该语句包含 else 部分，
则执行语句 2。

由于 if 语句只是简单测试表达式的数值，因此可以对某些代码的编写进行简化。最明显
的例子是用如下写法
  if (表达式) 
来代替
  if (表达式 !0) 
某些情况下这种形式是自然清晰的，但也有些情况下可能会含义不清。

因为 if-else 语句的 else 部分是可选的，所以在嵌套的 if 语句中省略它的 else 部分将导致歧义。解决的方法是将每个 else 与最近的前一个没有 else 配对的 if 进行匹配。可以使用花括号来明确地指定 else 与哪个 if 匹配。

### 3.3 else-if 语句

。。。k
if (表达式) 
  语句
else if (表达式) 
  语句
else if (表达式) 
  语句
else if (表达式) 
  语句
else 
  语句
。。。
因此我们在这里单独说明一下。这种 if 语句序列是编写多路判定最常用的方法。其中的各表达式将被依次求值，一旦某个表达式结果为真，则执行与之相关的语句，并终止整个语句序列的执行。同样，其中各语句既可以是单条语句，也可以是用花括号括住的复合语句。

最后一个 else 部分用于处理“上述条件均不成立”的情况或默认情况，也就是当上面各条件都不满足时的情形。有时候并不需要针对默认情况执行显式的操作，这种情况下，可以把该结构末尾的
`else 语句` 部分省略掉；该部分也可以用来检查错误，以捕获“不可能”的条件。

### 3.4 switch 语句
!!! note "switch 语句"
    switch 语句是一种多路判定语句，它测试表达式是否与一些常量整数值中的某一个值匹配，并执行相应的分支动作。

。。。k
switch (表达式) { 
  case 常量表达式: 语句序列
  case 常量表达式: 语句序列
  default: 语句序列
}
。。。

每一个分支都由一个或多个**整数值常量**或**常量表达式**标记。如果某个分支与表达式的值匹配，则从该分支开始执行。各分支表达式必须互不相同。如果没有哪一分支能匹配表达式，则执行标记为 default 的分支。default 分支是可选的。如果没有 default 分支也没有其它分支与表达式的值匹配，则该 switch 语句不执行任何动作。各分支及 default 分支的排列次序是任意的。

例子：

。。。k
#include <stdio.h>
main() /* count digits, white space, others */
{
    int c, i, nwhite, nother, ndigit[10];
    nwhite = nother = 0;
    for (i = 0; i < 10; i++)
        ndigit[i] = 0;
    while ((c = getchar()) != EOF)
    {
        switch (c)
        {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            ndigit[c - '0']++;
            break;
        case ' ':
        case '\n':
        case '\t':
            nwhite++;
            break;
        default:
            nother++;
            break;
        }
    }
    printf("digits =");
    for (i = 0; i < 10; i++)
        printf(" %d", ndigit[i]);
    printf(", white space = %d, other = %d\n",
           nwhite, nother);
    return 0;
}
。。。

break 语句将导致程序的执行立即从 switch 语句中退出。**在 switch 语句中，case的作用只是一个标号，因此，某个分支中的代码执行完后，程序将进入下一分支继续执行，除非在程序中显式地跳转。**跳出 switch 语句最常用的方法是使用 break 语句与 return 语句。break 语句还可强制控制从 while、for 与 do 循环语句中立即退出。

依次执行各分支的做法有优点也有缺点。好的一面是它可以把若干个分支组合在一起完成一个任务，如上例中对数字的处理。但是，正常情况下为了防止直接进入下一个分支执行，每个分支后必须以一个 break 语句结束。从一个分支直接进入下一个分支执行的做法并不健全，这样做在程序修改时很容易出错。除了一个计算需要多个标号的情况外，应尽量减少从一个分支直接进入下一个分支执行这种用法，在不得不使用的情况下应该加上适当的程序注释。

作为一种良好的程序设计风格，在 switch 语句最后一个分支（即 default 分支）的后面也加上一个 break 语句。这样做在逻辑上没有必要，但当我们需要向该 switch 语句后添加其它分支时，这种防范措施会降低犯错误的可能性。

### 3.5 while 循环与 for 循环

在 while 循环语句
。。。k
while (表达式) 
  语句
。。。
中，首先求表达式的值。如果其值非 0，则执行语句，并再次求该表达式的值。这一循环过程
一直进行下去，直到该表达式的值为 0 为止，随后继续执行语句后面的部分。

。。。k
for 循环语句; 
for (表达式 1; 表达式 2; 表达式 3) 
语句
。。。
等价于

。。。k
表达式 1;
while (表达式 2) {
  语句
  表达式 3;
}
。。。
!!! warning "注意"
    当 while 或 for 循环语句中包含 continue 语句时，上述二者之间就不一定等价了。

从语法角度看，for 循环语句的 3 个组成部分都是表达式。最常见的情况是，表达式 1
与表达式 3 是赋值表达式或函数调用，表达式 2 是关系表达式。这 3 个组成部分中的任何部
分都可以省略，但分号必须保留。如果在 for 语句中省略表达式 1 与表达式 3，它就退化成
了 while 循环语句。如果省略测试条件，即表达式 2，则认为其值永远是真值，因此，下列
for 循环语句：
。。。k
for ( ; ; ) 
  语句
。。。
是一个“无限”循环语句，这种语句需要借助其它手段（如 break 语句或 return 语句）才
能终止执行。

在设计程序时到底选用 while 循环语句还是 for 循环语句，主要取决于程序设计人员的个人偏好。

例如，在下列语句中：
。。。k
 while ((c = getchar()) == ' ' || c == '\n' || c = '\t') 
 ; /* skip white space characters */ 
。。。
因为其中没有初始化或重新初始化的操作，所以使用 whi1e 循环语句更自然一些。

如果语句中需要执行简单的初始化和变量递增，使用 for 语句更合适一些，它将循环控制语句集中放在循环的开头，结构更紧凑、更清晰。通过下列语句可以很明显地看出这一点：
。。。k
 for (i = 0; i < n; i++) 
 ...
。。。

例子，重写atoi函数：

。。。k
#include <ctype.h>
/* atoi: convert s to integer; version 2 */
int atoi(char s[])
{
    int i, n, sign;
    for (i = 0; isspace(s[i]); i++) /* skip white space */
        ;
    sign = (s[i] == '-') ? -1 : 1;
    if (s[i] == '+' || s[i] == '-') /* skip sign */
        i++;
    for (n = 0; isdigit(s[i]); i++)
        n = 10 * n + (s[i] - '0');
    return sign * n;
}
。。。

把循环控制部分集中在一起，对于多重嵌套循环，优势更为明显。下面的函数是对整型数组进行排序的 Shell 排序算法。Shell 排序算法是 D. L. Shell 于 1959 年发明的，其基本思想是：先比较距离远的元素，而不是像简单交换排序算法那样先比较相邻的元素。这样可以快速减少大量的无序情况，从而减轻后续的工作。被比较的元素之间的距离逐步减少，直到减少为 1，这时排序变成了相邻元素的互换。

。。。k
/* shellsort: sort v[0]...v[n-1] into increasing order */
void shellsort(int v[], int n)
{
    int gap, i, j, temp;
    for (gap = n / 2; gap > 0; gap /= 2)
        for (i = gap; i < n; i++)
            for (j = i - gap; j >= 0 && v[j] > v[j + gap]; j -= gap)
            {
                temp = v[j];
                v[j] = v[j + gap];
                v[j + gap] = temp;
            }
}
。。。

逗号运算符“,”也是 C 语言优先级最低的运算符，在 for 语句中经常会用到它。被逗号分隔的一对表达式将按照从左到右的顺序进行求值，表达式右边的操作数的类型和值即为其结果的类型和值。
。。。k
#include <string.h>
/* reverse: reverse string s in place */
void reverse(char s[])
{
    int c, i, j;
    for (i = 0, j = strlen(s) - 1; i < j; i++, j--)
    {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}
。。。
某些情况下的逗号并不是逗号运算符，比如分隔函数参数的逗号，分隔声明中变量的逗号等，这些逗号并不保证各表达式按从左至右的顺序求值。

应该慎用逗号运算符。逗号运算符最适用于关系紧密的结构中，比如上面的 reverse 函数内的 for 语句，对于需要在单个表达式中进行多步计算的宏来说也很适合。逗号表达式还适用于 reverse 函数中元素的交换，这样，元素的交换过程便可以看成是一个单步操作。

### 3.6 do-while 循环

我们在第 1 章中曾经讲过，while 与 for 这两种循环在循环体执行前对终止条件进行测试。与此相反，C 语言中的第三种循环——do-while 循环则在循环体执行后测试终止条件，这样循环体至少被执行一次。do-while 循环的语法形式如下：

。。。k
do 
  语句
while (表达式);
。。。

在这一结构中，先执行循环体中的语句部分，然后再求表达式的值。如果表达式的值为真，则再次执行语句，依此类推。当表达式的值变为假，则循环终止。

经验表明，do-while 循环比 while 循环和 for 循环用得少得多。尽管如此，do-while
循环语句有时还是很有用的。下面我们通过函数 itoa 来说明这一点。itoa 函数是 atoi 函数的逆函数，它把数字转换为字符串。这个工作比最初想像的要复杂一些。如果按照 atoi 函数中生成数字的方法将数字转换为字符串，则生成的字符串的次序正好是颠倒的，因此，我们首先要生成反序的字符串，然后再把该字符串倒置。

。。。k
/* itoa: convert n to characters in s */
void itoa(int n, char s[])
{
    int i, sign;
    if ((sign = n) < 0) /* record sign */
        n = -n;         /* make n positive */
    i = 0;
    do
    {                          /* generate digits in reverse order */
        s[i++] = n % 10 + '0'; /* get next digit */
    } while ((n /= 10) > 0);   /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}
。。。

### 3.7 break 与 continue 语句

不通过循环头部或尾部的条件测试而跳出循环，有时是很方便的。break 语句可用于从for、while与do-while等循环中提前退出，就如同从switch语句中提前退出一样。break语句能使程序从 switch 语句或最内层循环中立即跳出。

下面的函数 trim 用于删除字符串尾部的空格符、制表符与换行符。当发现最右边的字符为非空格符、非制表符、非换行符时，就使用 break 语句从循环中退出。

。。。k
/* trim: remove trailing blanks, tabs, newlines */
int trim(char s[])
{
    int n;
    for (n = strlen(s) - 1; n >= 0; n--)
        if (s[n] != ' ' && s[n] != '\t' && s[n] != '\n')
            break;
    s[n + 1] = '\0';
    return n;
}
。。。

strlen 函数返回字符串的长度。for 循环从字符串的末尾开始反方向扫描寻找第一个不是空格符、制表符以及换行符的字符。当找到符合条件的第一个字符，或当循环控制变量 n 变为负数时（即整个字符串都被扫描完时），循环终止执行。读者可以验证，即使字符串为空或仅包含空白符，该函数也是正确的。

continue 语句与 break 语句是相关联的，但它没有 break 语句常用。continue 语句用于使 for、while 或 do-while 语句开始下一次循环的执行。在 while 与 do-while语句中，continue 语句的执行意味着立即执行测试部分；在 for 循环中，则意味着使控制转移到递增循环变量部分。**continue 语句只用于循环语句，不用于 switch 语句。**某个循环包含的 switch 语句中的 continue 语句，将导致进入下一次循环。

当循环的后面部分比较复杂时，常常会用到 continue 语句。这种情况下，如果不使用continue 语句，则可能需要把测试颠倒过来或者缩进另一层循环，这样做会使程序的嵌套更深。

### 3.8 goto 语句与标号

C 语言提供了可随意滥用的 goto 语句以及标记跳转位置的标号。从理论上讲，goto 语句是没有必要的，实践中不使用 goto 语句也可以很容易地写出代码。至此，本书中还没有使用 goto 语句。

但是，在某些场合下 goto 语句还是用得着的。最常见的用法是终止程序在某些深度嵌套的结构中的处理过程，例如一次跳出两层或多层循环。这种情况下使用 break 语句是不能达到目的的，它只能从最内层循环退出到上一级的循环。下面是使用 goto 语句的一个例子：

。。。k
for (...)
    for (...)
    {
        ... if (disaster) goto error;
    }
... error:
    /* clean up the mess */
。。。

标号的命名同变量命名的形式相同，标号的后面要紧跟一个冒号。标号可以位于对应的
goto 语句所在函数的任何语句的前面。标号的作用域是整个函数。

我们来看另外一个例子。考虑判定两个数组 a 与 b 中是否具有相同元素的问题。一种可能的解决方法是：

。。。k
for (i = 0; i < n; i++)
    for (j = 0; j < m; j++)
        if (a[i] == b[j])
            goto found;
/* didn't find any common element */
... found :
    /* got one: a[i] == b[j] */
    ...
。。。

所有使用了 goto 语句的程序代码都能改写成不带 goto 语句的程序，但可能会增加一些额外的重复测试或变量。例如，可将上面判定是否具有相同数组元素的程序段改写成下列形式：

。。。k
found = 0;
for (i = 0; i < n && !found; i++)
    for (j = 0; j < m && !found; j++)
        if (a[i] == b[j])
            found = 1;
if (found)
/* got one: a[i-1] == b[j-1] */
... else
    /* didn't find any common element */
    ...
。。。

大多数情况下，使用 goto 语句的程序段比不使用 goto 语句的程序段要难以理解和维护，少数情况除外，比如我们前面所举的几个例子。尽管该问题并不太严重，但我们还是建议尽可能少地使用 goto 语句。

!!! note
  总之，别用 goto 语句。