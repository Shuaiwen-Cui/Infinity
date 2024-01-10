# C Language Study Notes

<div class="grid cards" markdown>

-   :fontawesome-solid-book:{ .lg .middle } __The C Programming Language🎯__

    ---
    Authors: Brian W. Kernighan and Dennis M. Ritchie

    [:octicons-arrow-right-24: <a href="https://kremlin.cc/k&r.pdf" target="_blank"> Download PDF </a>](#)

</div>

## Table of Contents

- [x] CH1 A Tutorial Introduction
- [x] CH2 Types, Operators, and Expressions
- [x] CH3 Control Flow
- [x] CH4 Functions and Program Structure
- [x] CH5 Pointers and Arrays
- [ ] CH6 Structures
- [ ] CH7 Input and Output
- [ ] CH8 The UNIX System Interface

## CH1 - A Tutorial Introduction

Some simple examples.

## CH2 - Types, Operators, and Expressions
- **Variables** and **constants** are the basic data objects manipulated in a program. 
- **Declarations** list the variables to be used, and state what type they have and perhaps what their initial values are. 
- **Operators** specify what is to be done to them. 
- **Expressions** combine variables and constants to produce new values. 
- The **type** of an object determines the set of values it can have and what operations can be performed on it.

### 2.1 Variable Names
There are some restrictions on the names of variables and symbolic constants:
- A name must begin with a letter.
  - Underscore is considered a letter, so it may appear in a name.
  - Since library routines and external variables begin with an underscore, it is better not to begin variable names with underscore.
- Upper and lower case letters are different.
  - By convention, variable names are lower case, and symbolic constants are upper case.
- The choice of variable names should be mnemonic.
  - Local variables may be short; one-letter names are ok for local variables, especially for loop indices.
  - External variables should have longer names.

### 2.2 Data Types and Sizes
C provides a basic set of data types:

- char: a single byte, capable of holding one character in the local character set.
- int: an integer, typically reflecting the natural size of integers on the host machine.
- float: single-precision floating point.
- double: double-precision floating point.

In addition, there are several qualifiers that can be applied to these basic types. The qualifier 'short' or 'long' can be applied to integers, and long can be applied to doubles. The effect is to either extend the size of the type or shorten it.

short int sh;
long int counter;

In these declarations, the keyword int can be omitted. It is usually omitted when the declaration contains a list of variables, as in the above example.

The introduction of 'short' and 'long' provides us with integers of different lengths to suit different needs. Integers are usually 16 bits long, long integers are usually 32 bits long, and short integers are usually 16 bits long. The compiler is free to choose appropriate sizes for its own hardware, subject only to the restriction that shorts and ints are at least 16 bits, longs are at least 32 bits, and short is no longer than int, which is no longer than long.

The qualifiers 'signed' and 'unsigned' may be applied to char or any integer. Unsigned numbers are always positive or zero, and obey the laws of arithmetic modulo 2^n, where n is the number of bits in the type. For example, if chars are 8 bits, unsigned char variables have values between 0 and 255, while signed chars have values between -128 and 127 (in a two's complement machine). Whether plain chars are signed or unsigned is machine-dependent, but printable characters are always positive.


### 2.3 Constants

#### Integer Constants

Integer constants is a type of int. If it is too large to fit in an int, it becomes a long. If it is suffixed with 'L' or 'l', it is a long. If it is suffixed with 'U' or 'u', it is an unsigned int. If it is suffixed with 'UL', 'Ul', 'uL', or 'ul', it is an unsigned long.

Apart from decimal, integer constants can be written in octal or hexadecimal. A leading 0 denotes octal; a leading 0x or 0X denotes hexadecimal. In octal and hexadecimal constants, the letters a through f and A through F represent values 10 through 15.

#### Floating Constants

Floating constants contain a decimal point (123.4) or an exponent (1e-2), or both. A floating constant without a suffix is double. A suffix f or F indicates float, and a suffix l or L indicates long double.

#### Character Constants

A character constant is a single character enclosed in single quotes. A string constant is a sequence of characters surrounded by double quotes. A character constant is just an integer, written as one character within single quotes, such as 'x'. The value of a character constant is the numeric value of the character in the machine's character set. For example, in the ASCII character set, the character constant '0' has the value 48, which is the internal representation of the character '0'.

!!! info 
    The complete set of escape sequences is:

    - \a: alert (bell) character

    - \b: backspace

    - \f: formfeed

    - \n: newline

    - \r: carriage return

    - \t: horizontal tab

    - \v: vertical tab

    - \\\\: backslash

    - \': single quote

    - \": double quote

    - \?: question mark

    - \ooo: octal number

    - \xhh: hexadecimal number

    - \0: null character

    - \0ddd: octal number

    - \xhh: hexadecimal number

Character constant '\0' represents the value 0 of the character, also known as null character. We usually use '\0' instead of 0 to emphasize the character attribute of some expressions, but its numeric value is 0.

Constant expressions are expressions that contain only constants. Such expressions are evaluated at compile time, rather than at run time. They may be used wherever constants are required.

#### String Constants

String constants, or string literals, are written as a sequence of characters enclosed in double quotes. A string constant is an array of characters. The internal representation of a string has a null character '\0' at the end, so the physical storage required is one more than the number of characters written between the quotes. This representation also explains why there is no limit on the length of a string; the program just has to scan memory until it finds a '\0' to determine the length. The standard library function strlen(s) returns the length of its character string argument, not counting the '\0'.

We should distinguish between a character constant and a one-character string. 'x' is a character constant, while "x" is a one-character string that contains the character x and a '\0'.

#### Enumeration Constants

Enumeration constants are a list of constant integer values. They are declared like this:

enum <enum name> {enum constant list};

The first enumeration constant has the value 0, and the value of each subsequent enumeration constant is increased by 1. The value of enumeration constants can be explicitly assigned when they are defined. If no explicit value is assigned, the value of an enumeration constant is one more than the value of the previous enumeration constant. The value of enumeration constants must be integer constants, and they cannot be repeated.

The scope of enumeration constants is the same as that of other variables, from the definition to the end of the block in which the enumeration constant is located. The type of enumeration constants is int.

### 2.4 Declarations

All variables must be declared before they can be used, although some variables can be declared implicitly by context. A declaration specifies a type, and contains a list of variables of that type.

A declaration can be split into multiple declarations in multiple declaration statements. Writing code in this way takes up more space, but it is easy to add comments to each declaration statement and to modify it later.

Variables can also be initialized when they are declared. In a declaration, if the variable name is followed by an equal sign and an expression, the expression serves as an initialization expression for the variable.

All variables must be declared before they are used, although some variables can be implicitly declared by context. A declaration specifies a variable type, followed by a variable list that can contain one or more variables of that type.
Multiple variables in a declaration statement can be broken up and declared in multiple declaration statements. Writing code in this form takes up more space, but it makes it easier to add comments to each declaration statement and to make changes later.
Variables can also be initialized at the same time as they are declared. In a declaration, if the variable name is followed by an equals sign and an expression, the expression serves as the initialization expression for initializing the variable.
If the variable is not an automatic variable, it can only be initialized once, conceptually before the program starts executing, and the initialization expression must be a constant expression. Each time a function or program block is entered, an explicitly initialized automatic variable is initialized once, and its initialization expression can be any expression. External variables and static variables are initialized to 0 by default. The value of an automatic variable that is not explicitly initialized is an undefined value (i.e., an invalid value).

The declaration of any variable can be qualified by the const qualifier. This qualifier specifies that the value of the variable cannot be changed. For arrays, the const qualifier specifies that the value of all elements of the array cannot be changed. The use of the const qualifier can improve the readability of the program and can also help the compiler check errors in the program.

### 2.5 Arithmetic Operators
The binary arithmetic operators include: +, -, *, /, and % (modulus operator). Integer division truncates the fractional part of the result. The result of the expression x % y is the remainder of x divided by y, and its value is 0 when x can be divided by y.

The modulus operator % cannot be applied to float or double types. In the case of negative operands, the direction of truncation of integer division and the sign of the modulus operator result depend on the implementation of the specific machine, just as they do in the case of overflow or underflow.

The binary operators + and - have the same priority, which is lower than the priority of the operators *, /, and %, and the priority of the operators *, /, and % is lower than the priority of the unary operators + and -. Arithmetic operators are associated from left to right.

### 2.6 Relational and Logical Operators
Relational operators include the following operators:
 \> >= < <=
They have the same priority. The priority of the equality operators is only lower than that of the relational operators:
 == !=
Relational operators have a lower priority than arithmetic operators.

Logical operators include the following operators:
&& ||

The logical operators && and || have some special properties. Expressions connected by && and || are evaluated from left to right, and, knowing the result value is true or false, the calculation is stopped immediately. Most C programs use these properties.

According to the definition, if the relationship is true in the relational expression or the logical expression, the result value of the expression is the numeric value 1; if it is false, the result value is the numeric value 0.

### 2.7 Type Conversions
When the operands of an operator have different types, they need to be converted to a common type using certain rules. Generally, automatic conversion refers to converting "narrower" operands to "wider" operands without losing information. Meaningless expressions are not allowed, for example, using a float expression as an index is not allowed. The compiler may give warning messages for expressions that may result in information loss, such as assigning a longer integer value to a shorter integer variable, assigning a floating-point value to an integer variable, and so on, but these expressions are not illegal.

Since the char type is a smaller integer type, char variables can be freely used in arithmetic expressions, which provides great flexibility for certain character conversions. For example, the following function atoi converts a string of digits into the corresponding numeric value.

When converting a character type to an integer type, we need to be careful. C language does not specify whether a char variable is unsigned or signed. When converting a char value to an int value, is it possible for the result to be a negative integer? The result varies for different machines, reflecting the differences between different machine architectures. In some machines, if the leftmost bit of the char value is 1, it is converted to a negative integer (performing "sign extension"). In other machines, when converting a char value to an int value, a 0 is added to the left of the char value, resulting in a positive value.

The C language definition guarantees that characters in the standard printing character set of the machine will not be negative values, so these characters are always positive in expressions. However, the bit pattern stored in a character variable can be negative on some machines and positive on others. To ensure program portability, it is best to specify the signed or unsigned qualifier if non-character data is stored in a char variable.

In C language, implicit arithmetic type conversion occurs in many cases. Generally, if the two operands of a binary operator (an operator with two operands, such as + or *) have different types, the "lower" type is promoted to the "higher" type before the operation, and the result of the operation is of the higher type.

Type conversion also occurs during assignment. The value on the right side of the assignment operator needs to be converted to the type of the variable on the left side, which is the type of the result of the assignment expression.

As mentioned earlier, whether or not sign extension is performed, character variables will be converted to integer variables. When converting a longer integer to a shorter integer or char type, the exceeding high-order bits will be discarded.

In any expression, you can use a unary operator called a cast operator to perform explicit type conversion. In the following statement, the expression is converted to the type specified by the type name according to the aforementioned conversion rules:
( type_name ) expression

### 2.8 Increment and Decrement Operators
C provides two special operators for incrementing and decrementing variables. The increment operator ++ increments its operand by 1, and the decrement operator decrements its operand by 1.

The special thing about the two operators ++ and -- is that they can be used as prefix operators (used before the variable, such as ++n). They can also be used as postfix operators (used after the variable, such as n++). In both cases, the effect is to add 1 to the value of the variable n. However, there is a slight difference between them. The expression ++n first increments the value of n by 1, and then uses the value of the variable n, while the expression n++ first uses the value of the variable n, and then increments the value of n by 1.

### 2.9 Bitwise Operators
C language provides 6 bitwise operators. These operators can only be applied to integer operands, specifically signed or unsigned char, short, int, long types:

- & (AND): Bitwise AND
- | (OR): Bitwise OR
- ^ (XOR): Bitwise XOR
- << (Left Shift): Left shift
- >> (Right Shift): Right shift
- ~ (Complement): Bitwise complement (unary operator)

The bitwise AND operator (&) is often used to mask certain binary bits, for example: n = n & 0177; This statement sets all bits in n except the 7 least significant bits to 0.

The bitwise OR operator (|) is commonly used to set certain binary bits to 1, for example: x = x | SET_ON; This statement sets the bits in x that correspond to the 1 bits in SET_ON to 1.

The bitwise XOR operator (^) sets the bit to 1 when the corresponding bits of the two operands are different, otherwise, it sets the bit to 0.

We must differentiate the bitwise operators (&, |) from the logical operators (&&, ||), which are used to evaluate the truth value of an expression from left to right. For example, if x is 1 and y is 2, the result of x & y is 0, while the value of x && y is 1.

The shift operators (<< and >>) are used to shift the left operand left or right by the number of positions specified by the right operand (which must be a non-negative value). Therefore, the expression x << 2 will shift the value of x 2 positions to the left, filling the 2 rightmost positions with 0. This expression is equivalent to multiplying the left operand by 4.

The unary operator (~) is used to obtain the bitwise complement of an integer, which means flipping each bit of the operand, changing 1s to 0s and 0s to 1s.


### 2.10 Assignment Operators and Expressions
For most binary operators (i.e., operators that have two operands, such as +), there is a corresponding assignment operator op=, where op can be one of the following operators:

`+ - * / % << >> & ^ |`

If expr1 and expr2 are expressions, then
`expr1 op= expr2`

is equivalent to:
`expr1 = (expr1) op (expr2)`

### 2.11 Conditional Expressions

The conditional expression `if-else` has the form:

```c
if (expression)
    statement
else
    statement
```

The trinary operator `?:` provides a convenient way to write a simple `if-else` statement. The expression

```c
expression1 ? expression2 : expression3
```

has the value of expression2 if expression1 is true (nonzero), and expression3 otherwise. Only one of the expressions expression2 and expression3 is evaluated.

### 2.12 Precedence and Order of Evaluation

The precedence of operators determines the order in which parts of an expression are evaluated. C has 15 precedence levels for operators. Operators in the same box have the same precedence. Operators in the same box are evaluated from left to right.

| Operator | Meaning | Associativity |
| :---: | :---: | :---: |
| () [] -> . | Parentheses, brackets, dot, arrow | Left to right |
| ! ~ ++ -- + - * & (type) sizeof | Unary operators | Right to left |
| * / % | Multiplication, division, remainder | Left to right |
| + - | Addition, subtraction | Left to right |
| << >> | Left shift, right shift | Left to right |
| < <= > >= | Relational operators | Left to right |
| == != | Equality operators | Left to right |
| & | Bitwise AND | Left to right |
| ^ | Bitwise XOR | Left to right |
| \| | Bitwise OR | Left to right |
| && | Logical AND | Left to right |
| \|\| | Logical OR | Left to right |
| ?: | Conditional expression | Right to left |
| = += -= *= /= %= <<= >>= &= ^= \|= | Assignment | Right to left |
| , | Comma | Left to right |

## Chapter 3 - Control Flow
Control flow statements in programming languages are used to manage the sequence of execution for various computational operations.

### 3.1 Statements and Code Blocks

#### Statements
Expressions like `x = 0`, `i++`, or `printf(...)` become statements when followed by a semicolon (`;`). In the C language, the semicolon serves as the statement terminator, while other languages like Pascal use it as a separator between statements.

#### Code Blocks
A group of declarations and statements enclosed in curly braces `{` and `}` forms a compound statement (also known as a code block). A compound statement is syntactically equivalent to a single statement. The right curly brace marks the end of the code block and does not require a semicolon.

### 3.2 if-else Statement

```c
if {expression} 
  statement1
else 
  statement2
```
The `else` part is optional in this statement. When the statement is executed, the expression is first evaluated. If the expression is true (i.e., the value is non-zero), then statement 1 is executed. If the expression is false (i.e., the value is 0), and the statement contains an `else` part, then statement 2 is executed.

Since the `if` statement simply tests the numerical value of the expression, the code can be simplified in some cases. For example, using the form:
```c
if (expression)
```
is equivalent to:
```c
if (expression != 0)
```
In some cases, this form is naturally clear, but in others, it may be less clear in meaning.

Because the else part of the if-else statement is optional, omitting the else part in nested if statements can lead to ambiguity. The solution is to match each else with the nearest preceding if that does not have an else. Curly braces can be used to explicitly specify which if the else corresponds to.

### 3.3 else-if Statement

```c
if (expression1)
  statement1
else if (expression2)
  statement2
else if (expression3)
  statement3
else
  statement4
```

Therefore, we explain this separately here. This kind of `if` statement sequence is the most commonly used method for writing multi-way decisions. The expressions within it are evaluated sequentially, and once a certain expression evaluates to true, the associated statement is executed, and the execution of the entire statement sequence is terminated. Similarly, the statements within it can be either single statements or compound statements enclosed in curly braces.

The last `else` part is used to handle the case when "none of the above conditions are true" or the default case, which is when none of the above conditions are satisfied. Sometimes, it is not necessary to perform explicit operations for the default case. In such cases, the `else` statement at the end of this structure can be omitted. This part can also be used to check for errors and capture "impossible" conditions.

### 3.4 switch Statement
!!! note "switch Statement"
    The switch statement is a multi-way decision statement that tests whether an expression matches one of several constant integer values and executes the corresponding branch action.

```c
switch (expression) {
  case constant1:
    statement1
    break;
  case constant2:
    statement2
    break;
  ...
  default:
    statement
}
```
Each branch is marked by one or more **integer constants** or **constant expressions**. If a branch matches the value of the expression, execution begins from that branch. The expressions for each branch must be mutually exclusive. If none of the branches matches the expression's value, the branch marked as `default` is executed. The `default` branch is optional. If there is no `default` branch and no other branch matches the expression's value, the switch statement takes no action. The arrangement order of branches and the `default` branch is arbitrary.

Example:
```c
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
```

The `break` statement causes an immediate exit from the switch statement. **In a switch statement, the role of a `case` is merely a label. Therefore, after the code in a branch is executed, the program proceeds to the next branch unless explicitly redirected in the program.** The most common ways to exit a switch statement are using the `break` and `return` statements. The `break` statement can also be used to force an immediate exit from `while`, `for`, and `do` loop statements.

Executing branches sequentially has both advantages and disadvantages. The benefit is that it allows combining several branches to accomplish a task, such as handling numbers in the example above. However, to prevent directly entering the next branch in normal circumstances, each branch must end with a `break` statement. Directly entering the next branch without a `break` statement is not a sound practice and can lead to errors during program modification. Except in cases where a calculation requires multiple labels, this usage of moving directly from one branch to the next should be minimized, and if necessary, appropriate program comments should be added.

As a good programming practice, it is advisable to include a `break` statement after the last branch (i.e., the `default` branch) in a switch statement. While this is not logically necessary, it serves as a preventive measure against potential errors when adding other branches after this switch statement.

### 3.5 Loops - while and for

In while loop

```c
while (expression)
  statement
```
First, evaluate the expression. If its value is non-zero, execute the statement and then reevaluate the expression. This looping process continues until the value of the expression becomes 0, after which the execution proceeds to the rest of the statement.

```c
for loopexpressions;
for (expr1; expr2; expr3)
  statement
```

is equivalent to:

```c
expr1;
while (expr2) {
  statement
  expr3;
}
```

!!! warning "注意"
    When the `continue` statement is present within a `while` or `for` loop statement, the equivalence between the two is not guaranteed.

From a syntax perspective, the three components of the `for` loop statement are expressions. The most common scenario is that expressions 1 and 3 are assignment expressions or function calls, and

```c
for (;;)
  statement
```

is an "infinite" loop statement, and such a statement requires other means (such as the `break` statement or `return` statement) to terminate its execution.

Whether to use a `while` loop statement or a `for` loop statement in program design primarily depends on the personal preference of the programmer.

For example, in the following statements:
```c
 while ((c = getchar()) == ' ' || c == '\n' || c = '\t') 
 ; /* skip white space characters */ 
```
Because there is no initialization or reinitialization operation in it, using a `while` loop statement is more natural.

If the statement requires simple initialization and variable increment, using a `for` statement is more appropriate. It consolidates the loop control statements at the beginning of the loop, making the structure more concise and clear. This is evident in the following statement:

```c
 for (i = 0; i < n; i++) 
 ...
```

let's rewrite the atoi function:

```c
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
```

To consolidate the control flow for loops, especially in the case of nested loops, it is advantageous to centralize the control section. The following function demonstrates the Shell Sort algorithm for sorting an array of integers. Shell Sort was invented by D. L. Shell in 1959, and its basic idea is to initially compare elements that are far apart, rather than adjacent elements as in simple exchange sorting algorithms. This approach quickly reduces a significant amount of disorder, thereby easing subsequent work. The distance between the compared elements gradually decreases until it becomes 1, at which point the sorting turns into the exchange of adjacent elements.

```c
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
```

The comma operator "," is also the lowest precedence operator in the C language and is often used in for statements. A pair of expressions separated by a comma are evaluated from left to right, and the type and value of the right operand determine the type and value of the result.
```c
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
```

In certain situations, the comma is not the comma operator, such as when separating function parameters or variables in declarations. In these cases, the commas do not guarantee that the expressions will be evaluated from left to right.

Caution should be exercised when using the comma operator. It is most suitable for tightly related structures, such as the for statement inside the reverse function mentioned earlier. It is also well-suited for macros that require multi-step calculations wi

### 3.6 do-while Loop

As mentioned in Chapter 1, both the while and for loops in C test the termination condition before executing the loop body. In contrast, the third type of loop in the C language—the do-while loop—tests the termination condition after executing the loop body, ensuring that the loop body is executed at least once. The syntax of the do-while loop is as follows:

```c
do
  statement
while (expression);
```

In this structure, the statements in the loop body are executed first, followed by the evaluation of the expression. If the expression evaluates to true, the statements are executed again, and this process continues. The loop terminates when the expression evaluates to false.

Experience indicates that do-while loops are used much less frequently than while and for loops. Nevertheless, the do-while loop statement can be useful in certain situations. We illustrate this throug

```c
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
```
### 3.7 break and continue Statements

Breaking out of a loop without waiting for the condition at the loop header or footer to be tested can be convenient. The break statement is used to prematurely exit loops such as for, while, and do-while, similar to prematurely exiting a switch statement. The break statement allows the program to immediately exit from the switch statement or the innermost loop.

The following trim function is used to remove trailing spaces, tabs, and newline characters from a string. The break statement is employed to exit the loop when a non-space, non-tab, non-newline character is encountered at the rightmost position.

```c
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
```

The strlen function returns the length of a string. The for loop scans in reverse from the end of the string to find the first character that is not a space, tab, or newline. The loop terminates when the first satisfying character is found or when the loop control variable n becomes negative (indicating that the entire string has been scanned). Readers can verify that even if the string is empty or contains only whitespace characters, the function works correctly.

The continue statement is related to the break statement but is less commonly used. The continue statement is used to skip the rest of the current iteration and start the next iteration of a for, while, or do-while loop. In while and do-while loops, the execution of the continue statement means immediately evaluating the test part; in a for loop, it means transferring control to the incrementing loop variable part. **The continue statement is only used within loop statements and not within switch statements.** The use of continue in a switch statement within a loop leads to the next iteration of the loop.

When the later part of a loop is relatively complex, the continue statement is often used. In such cases, without using the continue statement, it might be necessary to reverse the test or indent another loop layer, resulting in deeper program nesting.

### 3.8 goto Statement and Labels

C language provides the goto statement and labels for arbitrary jumps. In theory, the goto statement is unnecessary, and in practice, it's possible to write code easily without using goto. Up to this point in the book, goto statements have not been used.

However, there are situations where the goto statement can be useful. The most common use is to terminate the processing in a deeply nested structure, such as breaking out of two or more layers of loops at once. In such cases, using the break statement is insufficient as it only exits the innermost loop. Here is an example using the goto statement:

```c
for (...)
    for (...)
    {
        ... if (disaster) goto error;
    }
... error:
    /* clean up the mess */
```

Labels are named in the same way as variables, followed immediately by a colon. Labels can be placed before any statement in the function where the corresponding goto statement is present. The scope of a label is the entire function.

Let's consider another example. Suppose we want to determine whether two arrays, a and b, have the same elements. One possible solution is:

```c
for (i = 0; i < n; i++)
    for (j = 0; j < m; j++)
        if (a[i] == b[j])
            goto found;
/* didn't find any common element */
... found :
    /* got one: a[i] == b[j] */
    ...
```

All programs using the goto statement can be rewritten without goto, although this may lead to additional redundant tests or variables. For example, the segment of code determining whether two arrays have the same elements, as shown above, can be rewritten as follows:

```c
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
```

In most cases, code segments using the goto statement are harder to understand and maintain compared to those without goto, with few exceptions, such as the examples mentioned earlier. While this issue may not be severe, it is still recommended to use the goto statement sparingly.

!!! note
    In general, avoid using the goto statement.

## Chapter 4 - Functions and Program Structure

Functions enable the decomposition of large computing tasks into smaller, more manageable ones. Programmers can further construct programs based on functions without the need to rewrite code. Well-designed functions can abstract away specific operational details, making the overall program structure clearer and reducing the difficulty of modifying the program. In the design of the C language, both the efficiency and usability of functions were taken into consideration. C language programs typically consist of numerous small functions rather than a few large ones, emphasizing modularity and code organization.

### 4.1 Basics of Functions

```c
#include <stdio.h>
#define MAXLINE 1000 /* maximum input line length */
int getline(char line[], int max) int strindex(char source[], char searchfor[]);
char pattern[] = "ould"; /* pattern to search for */
/* find all lines matching pattern */
main()
{
    char line[MAXLINE];
    int found = 0;
    while (getline(line, MAXLINE) > 0)
        if (strindex(line, pattern) >= 0)
        {
            printf("%s", line);
            found++;
        }
    return found;
}
/* getline: get line into s, return length */
int getline(char s[], int lim)
{
    int c, i;
    i = 0;
    while (--lim > 0 && (c = getchar()) != EOF && c != '\n')
        s[i++] = c;
    if (c == '\n')
        s[i++] = c;
    s[i] = '\0';
    return i;
}
/* strindex: return index of t in s, -1 if none */
int strindex(char s[], char t[])
{
    int i, j, k;
    for (i = 0; s[i] != '\0'; i++)
    {
        for (j = i, k = 0; t[k] != '\0' && s[j] == t[k]; j++, k++)
            ;
        if (k > 0 && t[k] == '\0')
            return i;
    }
    return -1;
}
```

The format for defining a function is as follows:

```c
return_type function_name(parameter_declaration_list) 
{
    declarations and statements
}
```

Each component in the function definition can be omitted. The simplest function looks like this:

```c
void dummy() {}
```
The function `dummy` does not perform any operation and does not return any value. Such functions, which do not execute any specific operations, are sometimes useful during program development to reserve a space for future code additions. If the return type is omitted in the function definition, it defaults to the `int` type.

A program can be considered as a collection of variable definitions and function definitions. Communication between functions can occur through parameters, return values, and external variables. The order of function appearances in the source file can be arbitrary, as long as each function is not split across multiple files. The source program can be divided into multiple files.

A called function returns a value to the caller through the `return` statement, where the statement can be followed by any expression:
```c
return expression;
```
If necessary, the expression will be converted to the return value type of the function. Parentheses are typically placed around the expression on both sides, though they are optional.

Calling functions can ignore the return value. Additionally, the `return` statement may not necessarily require an expression. When there is no expression following the `return` statement, the function does not return a value to the caller. When a called function reaches the final right curly brace and completes execution, control is also returned to the caller (without returning a value). If a function returns a value from one point and not from another, it is not illegal but may indicate a potential issue. In any case, if a function fails to return a value successfully, its "value" is certainly useless.

### 4.2 Functions Returning Non-integers

```c
#include <ctype.h>
/* atof: convert string s to double */
double atof(char s[])
{
    double val, power;
    int i, sign;
    for (i = 0; isspace(s[i]); i++) /* skip white space */
        ;
    sign = (s[i] == '-') ? -1 : 1;
    if (s[i] == '+' || s[i] == '-')
        i++;
    for (val = 0.0; isdigit(s[i]); i++)
        val = 10.0 * val + (s[i] - '0');
    if (s[i] == '.')
        i++;
    for (power = 1.0; isdigit(s[i]); i++)
    {
        val = 10.0 * val + (s[i] - '0');
        power *= 10;
    }
    return sign * val / power;
}
```

```c
#include <stdio.h>
#define MAXLINE 100
/* rudimentary calculator */
main()
{
    double sum, atof(char[]);
    char line[MAXLINE];
    int getline(char line[], int max);
    sum = 0;
    while (getline(line, MAXLINE) > 0)
        printf("\t%g\n", sum += atof(line));
    return 0;
}
```

!!! tip
    If a function has parameters, they should be declared; if there are no parameters, declare using `void`.

### 4.3 External Variables

C language programs can be viewed as a series of external objects, which can be variables or functions. The adjectives "external" and "internal" are used in contrast, with "internal" describing function parameters and variables defined within a function. External variables are defined outside functions, allowing them to be used in multiple functions. Since C does not permit the definition of other functions within a function, functions themselves are considered "external." By default, external variables and functions share the property that **all references to an external variable by the same name (even if these references come from different functions compiled separately) actually refer to the same object (referred to as external linkage in the standard)**.

Because external variables can be accessed globally, they provide an alternative way for data exchange between functions instead of using function parameters and return values. Any function can access an external variable by name, which needs to be declared in some way.

!!! warning
    External variables can increase coupling between programs, so their usage should be minimized.

The utility of external variables is also evident in their larger scope and longer lifespan compared to internal variables. Automatic variables can only be used within a function, existing from the moment the function is called until it exits. In contrast, external variables persist permanently, and their values remain unchanged from one function call to the next. Therefore, if two functions need to share certain data and do not call each other, the most convenient approach is to define these shared data as **external variables** rather than passing them as **function parameters**.

!!! info
    Reverse Polish Notation (RPN), also known as postfix notation (operators are written after their operands). RPN does not require parentheses; it only needs to know how many operands each operator requires to avoid ambiguity.

!!! note
    Calculator Implementation: The implementation of a calculator program is straightforward. Each operand is sequentially pushed onto the stack. When an operator is encountered, the corresponding number of operands (two operands for binary operators) are popped from the stack, the operator is applied to the popped operands, and the result is pushed back onto the stack.

```c
while (next operator or operand is not the end-of-file indicator)
{
    if (it is a number)
    {
        push the number onto the stack;
    }
    else if (it is an operator)
    {
        pop the required number of operands;
        perform the operation;
        push the result onto the stack;
    }
    else if (it is a newline character)
    {
        pop and print the value at the top of the stack;
    }
    else
    {
        // Handle error
    }
}
```
The push and pop operations for the stack are relatively simple, but if error detection and recovery operations are included, the program can become lengthy. It is advisable to design independent functions for error handling and recovery, rather than using them as repetitive code segments in the program. Additionally, a separate function is needed to fetch the next input operator or operand.

So far, we haven't discussed a crucial aspect of the design: where to place the stack? In other words, which routines can directly access it? One possibility is to place it in the main function, passing the stack and its current position as parameters to functions that perform push or pop operations on it. However, the main function doesn't need to be aware of the stack's variable details; it only performs push and pop operations. Therefore, the stack and related information can be stored in external variables, accessible only to the push and pop functions, not to the main function.

Translating the above paragraph into code is straightforward. If the program is in a single source file, it might look something like the following:

```c
```c
#include <...>   // Some included header files
#define ...      // Some define definitions

// Function declarations used by main
main() {
    // ...

    void push(double f);
    double pop(void);
    int getop(char s[]);

    // ...
}

// External variables used by push and pop
void push(double f) {
    // ...
}

double pop(void) {
    // ...
}

// Function called by getop
int someFunction(void) {
    // ...
}
```
In the later sections, we will discuss how to split this program into two or more source files.

The `main` function includes a large `switch` loop, which controls the program flow based on the type of operator or operand. The usage of the `switch` statement here is more typical than the example in Section 3.4.
```c
#include <stdio.h>
#include <stdlib.h> /* for atof() */
#define MAXOP 100   /* max size of operand or operator */
#define NUMBER '0'  /* signal that a number was found */
int getop(char[]);
void push(double);
double pop(void);
/* reverse Polish calculator */
main()
{
    int type;
    double op2;
    char s[MAXOP];
    while ((type = getop(s)) != EOF)
    {
        switch (type)
        {
        case NUMBER:
            push(atof(s));
            break;
        case '+':
            push(pop() + pop());
            break;
        case '*':
            push(pop() * pop());
            break;
        case '-':
            op2 = pop();
            push(pop() - op2);
            break;
        case '/':
            op2 = pop();
            if (op2 != 0.0)
                push(pop() / op2);
            else
                printf("error: zero divisor\n");
            break;
        case '\n':
            printf("\t%.8g\n", pop());
            break;
        default:
            printf("error: unknown command %s\n", s);
            break;
        }
    }
    return 0;
}
```
Because the `+` and `*` operators satisfy the commutative property, the order of popping operands is not crucial. However, for the `-` and `/` operators, the left and right operands must be distinguished. The order of evaluation for the two `pop` calls is not defined in a function call. To ensure the correct order, the first value must be popped into a temporary variable, similar to what is done in the `main` function.

```c
push(pop() - pop()); /* WRONG */
```
```c
#define MAXVAL 100  /* maximum depth of val stack */
int sp = 0;         /* next free stack position */
double val[MAXVAL]; /* value stack */
/* push: push f onto value stack */
void push(double f)
{
    if (sp < MAXVAL)
        val[sp++] = f;
    else
        printf("error: stack full, can't push %g\n", f);
}
/* pop: pop and return top value from stack */
double pop(void)
{
    if (sp > 0)
        return val[--sp];
    else
    {
        printf("error: stack empty\n");
        return 0.0;
    }
}
```
If a variable is defined outside of any function, it is an external variable. Therefore, we define the stack and stack pointer, which must be shared by the `push` and `pop` functions, externally.

Now, let's take a look at the implementation of the `getop` function. This function retrieves the next operator or operand. The task is relatively straightforward. It needs to skip spaces and tabs. If the next character is not a digit or a decimal point, it returns; otherwise, it collects the string of digits (which may include a decimal point) and returns `NUMBER` to indicate that a number has been collected.

```c
#include <ctype.h>
int getch(void);
void ungetch(int);
/* getop: get next character or numeric operand */
int getop(char s[])
{
    int i, c;
    while ((s[0] = c = getch()) == ' ' || c == '\t')
        ;
    s[1] = '\0';
    if (!isdigit(c) && c != '.')
        return c; /* not a number */
    i = 0;
    if (isdigit(c)) /* collect integer part */
        while (isdigit(s[++i] = c = getch()))
            ;
    if (c == '.') /* collect fraction part */
        while (isdigit(s[++i] = c = getch()))
            ;
    s[i] = '\0';
    if (c != EOF)
        ungetch(c);
    return NUMBER;
}
```
The `getch` and `ungetch` functions in this program serve a specific purpose. Often in programs, there is a situation where the program cannot determine if it has read enough input unless it reads a bit more ahead. One example is when reading characters to compose a number: before encountering the first non-digit character, the completeness of the read number cannot be guaranteed. Since the program needs to read one extra character ahead, it leads to having one character that doesn't belong to the current number being read.

This problem can be addressed by "unreading" the unnecessary characters. Whenever a program reads an extra character, it can push it back into the input stream, making it as if the character was not read for the rest of the code. We can write a pair of collaborating functions to conveniently simulate this "ungetting" operation. The `getch` function is used to read the next character to be processed, while the `ungetch` function is used to put a character back into the input stream. Subsequently, when calling the `getch` function, it returns the character put back by the `ungetch` function before reading new input.

```c
#define BUFSIZE 100
char buf[BUFSIZE]; /* buffer for ungetch */
int bufp = 0;      /* next free position in buf */
int getch(void)    /* get a (possibly pushed-back) character */
{
    return (bufp > 0) ? buf[--bufp] : getchar();
}
void ungetch(int c) /* push character back on input */
{
    if (bufp >= BUFSIZE)
        printf("ungetch: too many characters\n");
    else
        buf[bufp++] = c;
}
```

### 4.4 Scope Rules

Functions and external variables that constitute a C language program can be compiled separately. A program can be stored in multiple files, and precompiled functions can be loaded from libraries.

- How should declarations be made to ensure that variables are correctly declared at compile time?

- How should the positions of declarations be arranged to ensure that different parts of the program can be correctly linked when loaded?

- How should declarations in the program be organized to ensure that there is only one copy?

- How are external variables initialized?

The **scope** of a name refers to the part of the program where that name can be used. For automatic variables declared at the beginning of a function, their scope is the function where the variable is declared. There is no relationship between local variables with the same name declared in different functions. The same applies to function parameters; they can be considered as **local variables**.

The scope of an **external variable** or function begins where it is declared and ends at the end of the (to-be-compiled) file in which it resides.

For example, if `main`, `sp`, `val`, `push`, and `pop` are sequentially defined as 5 functions or external variables in a file, it might look like this:
```c
main() { ... }
int sp = 0;
double val[MAXVAL];
void push(double f) { ... }
double pop(void) { ... }
```
So, in the `push` and `pop` functions, variables `sp` and `val` can be accessed by name without any declarations. However, these variable names cannot be used in the `main` function, and the `push` and `pop` functions cannot be used in the `main` function.

On the other hand, if you want to use a variable before its external variable definition or if the external variable definition and usage are not in the same source file, you must forcefully use the `extern` keyword in the respective variable declarations.

!!! note Variables

    Local variables

    External variables (Global variables)

    External external variables (Declared using `extern`)

It is crucial to strictly distinguish between the **declaration** and **definition** of external variables. A **variable declaration** is used to specify the **attributes of the variable** (primarily the variable's type), while a **variable definition** not only does this but also causes **memory allocation**.

In all source files of a source program, **an external variable can only be defined once in a specific file**, and other files can access it through an `extern` declaration. The source file defining the external variable may also include an `extern` declaration for that variable. The definition of an external variable must specify the array's length, but an `extern` declaration does not necessarily need to specify the array's length.

!!! note
    `extern` is used in two scenarios:

    (1) Use before definition (can also be omitted in the same file)

    (2) Sharing variables across multiple files (most common case)

**Initialization of external variables must occur in their definition.**

### 4.5 Header Files
Now, let's consider the scenario of splitting the calculator program into several source files. If various components of the program are lengthy, it becomes necessary to do so. Here's how we split it:

- Place the main function `main` in a separate file `main.c`.
- Place the `push` and `pop` functions along with their external variables in a second file `stack.c`.
- Place the `getop` function in a third file `getop.c`.
- Place the `getch` and `ungetch` functions in a fourth file `getch.c`.

The reason for splitting into multiple files is primarily considering that, in real-world programs, these components might come from separately compiled libraries.

Additionally, we must consider the issue of sharing definitions and declarations between these files. We try to concentrate shared portions as much as possible so that only one copy is needed, making it easier to ensure correctness when improving the program.

![code organization](CH4-5-code-organization.png)

We make a compromise between two factors: on one hand, we want each file to access only the information it needs to complete its task; on the other hand, maintaining many header files in reality can be challenging. A conclusion we can draw is that for medium-sized programs, it's best to use a single header file to store objects shared among different parts of the program. Larger programs may require more header files, and careful organization is needed.

For certain variables, such as the variables `sp` and `val` defined in the file `stack.c`, and the variables `buf` and `bufp` defined in the file `getch.c`, they are intended for use only by functions within their respective source files, and other functions should not have access to them. Using the `static` declaration to qualify external variables and functions **can restrict the scope of objects declared later to the remaining part of the compiled source file**. **By using `static` to qualify external objects, the goal of hiding external objects can be achieved**, for example, the composite structure of `getch-ungetch` needs to share the variables `buf` and `bufp`. Thus, `buf` and `bufp` must be external variables, but these two objects should not be accessible to callers of the `getch` and `ungetch` functions.

To designate an object as having static storage duration, you can add the `static` keyword as a prefix before the regular object declaration. If these two functions and two variables are compiled in one file, it would look like this:

```c
static char buf[BUFSIZE]; /* buffer for ungetch */
static int bufp = 0;      /* next free position in buf */

int getch(void) { ... }
void ungetch(int c) { ... }
```

Now, other functions cannot access the variables `buf` and `bufp`. Therefore, these two names will not conflict with the same names in other files within the same program. Similarly, the variables `sp` and `val` can be declared as static to hide these variables used by the `push` and `pop` functions, which manipulate the execution stack.

External static declarations are commonly used for variables, and they can also be used for function declarations. Generally, function names are globally accessible and visible to all parts of the program. **However, if a function is declared as static, the function name is visible only in the file where the declaration is made, and other files cannot access it.**

### 4.7 Register Variables
The `register` declaration informs the compiler that the declared variable is frequently used in the program. The idea is to place register variables in the machine's registers, making the program smaller and faster. However, the compiler may ignore this option.

### 4.8 Program Block Structure
In a function, variables can be defined using a program block structure. Variable declarations (including initialization) can follow not only immediately after the opening brace of the function but also after the opening brace of any other compound statement. Variables declared in this way can hide variables with the same name outside the program block, and there is no relationship between them. They exist until the right brace matching the left brace appears.

Each time a program block is entered, automatic variables declared and initialized within the block will be initialized. Static variables are initialized only once when entering the program block for the first time.

Automatic variables (including formal parameters) can also hide external variables and functions with the same name.

!!! tip
    In good programming style, one should avoid situations where variable names in a local scope hide names from an external scope. Otherwise, confusion and errors are likely to occur.

### 4.9 Initialization
Without explicit initialization, external and static variables are initialized to 0, while the initial value of automatic and register variables is undefined (i.e., contains garbage values).

When defining a scalar variable, you can initialize it by placing an equal sign and an expression immediately after the variable name.

For external and static variables, the initialization expression must be a constant expression, and it is initialized only once (conceptually at the beginning of program execution). For automatic and register variables, it is initialized every time the function or program block is entered.

For automatic and register variables, the initialization expression does not have to be a constant expression; the expression can include any values defined before this expression, including function calls.

In practice, the initialization of automatic variables is equivalent to a shorthand assignment statement.

Arrays can be initialized by following the declaration with an initializer list enclosed in braces, with each initialization expression separated by commas.

Initialization of character arrays is special: you can use a string instead of an initializer list enclosed in braces and separated by commas.

### 4.10 Recursion
In the C programming language, functions can be recursively called, meaning a function can directly or indirectly invoke itself.

```c
#include <stdio.h>
/* printd: print n in decimal */
void printd(int n)
{
    if (n < 0)
    {
        putchar('-');
        n = -n;
    }
    if (n / 10)
        printd(n / 10);
    putchar(n % 10 + '0');
}
```
When a function recursively calls itself, each invocation creates a new set of automatic variables distinct from the previous ones. Therefore, when calling `printd(123)`, the first invocation of `printd` has the parameter `n=123`. It passes 12 to the second invocation of `printd`, which in turn passes 1 to the third invocation of `printd`. In the third invocation, it first prints 1 and then returns to the second invocation. After returning from the third invocation, the second invocation will print 2 and then return to the first invocation. Upon returning to the first invocation, it prints 3 and completes the execution of the function.

Recursion does not save memory overhead because there must be a stack to maintain the values during the recursive calls. Recursive execution is not necessarily fast, but recursive code is more compact and often easier to write and understand than the corresponding non-recursive code.

### 4.11 The C Preprocessor

### 4.11 C Language Preprocessor

C language provides certain language functionalities through the preprocessor. Conceptually, the preprocessor is the initial step executed independently in the compilation process. The two most commonly used preprocessor directives are `#include` (used to include the content of a specified file into the current file during compilation) and `#define` (used to substitute any character sequence for a token). This section will also cover some other features of the preprocessor, such as conditional compilation and parameterized macros.


#### 4.11.1 File Inclusion
The file inclusion directive (`#include`) makes it more convenient to deal with a large amount of `#define` statements and declarations. In a source file, any line like `#include "filename"` or `#include <filename>` will be replaced by the content of the file specified by `filename`. If the filename is enclosed in double quotes, the file is searched for at the location of the source file; if the file is not found there, or if the filename is enclosed in angle brackets `<` and `>`, the file is searched for according to specific rules, depending on the implementation. The included file itself can also contain `#include` directives.

Usually, multiple `#include` directives appear at the beginning of a source file, containing common `#define` statements and `extern` declarations, or function prototype declarations for library functions accessed from header files like `<stdio.h>`.


#### 4.11.2 Macro Substitution
Macro definitions have the form `#define name replacement_text`. This is the simplest form of macro substitution—every subsequent occurrence of the token `name` will be replaced by `replacement_text`.

In general, a `#define` directive occupies one line, and the replacement text is the remaining content at the end of the `#define` directive line. However, a longer macro definition can be split into several lines, in which case a backslash `\` is placed at the end of lines to be continued. The scope of the name defined by a `#define` directive starts at the point of definition and extends to the end of the source file being compiled.

Macro definitions can also use previously defined macros. The replacement only applies to tokens, and strings enclosed in quotes are not affected. For example, if `YES` is a name defined by a `#define` directive, it will not be replaced in statements like `printf("YES")` or `YESMAN`.

Replacement text can be anything; for example, `#define forever for (;;) /* infinite loop */` defines a new name `forever` for an infinite loop.

Macro definitions can also have parameters, allowing different replacement texts for different macro invocations. For instance, the following macro definition defines a `max` macro: `#define max(A, B) ((A) > (B) ? (A) : (B))`

Using the `max` macro looks similar to a function call, but the macro invocation directly inserts the replacement text into the code. Each occurrence of formal parameters (here, A or B) will be replaced with the corresponding actual parameters. Thus, the statement `x = max(p+q, r+s);` will be replaced with: `x = ((p+q) > (r+s) ? (p+q) : (r+s));`

Some flaws exist in the expanded form of `max`. Expressions acting as parameters need to be evaluated twice, and if expressions have side effects (e.g., involving increment operators or I/O), incorrect results may occur. For example, `max(i++, j++) /* WRONG */` leads to two increment operations for each parameter. Additionally, it is crucial to use parentheses properly to ensure correct order of evaluation. Consider the following macro definition: `#define square(x) x * x /* WRONG */`—what happens when the macro `square(z+1)` is called?

The `#undef` directive can be used to cancel the definition of a macro, ensuring that subsequent calls are treated as function calls rather than macro calls.

Parameters cannot be replaced with quoted strings. However, if the parameter name is prefixed with `#` in the replacement text, the result will be expanded to the quoted string of the actual parameter.


#### 4.11.3 Conditional Inclusion
Conditional statements can also be used to control the preprocessor itself, and these conditional statements are evaluated during preprocessing. This allows selective inclusion of different code based on calculated conditional values during compilation.

The `#if` statement evaluates constant integer expressions (which cannot contain `sizeof`, type cast operators, or enum constants). If the value of this expression is non-zero, the lines following the `#if` statement up to encountering `#endif`, `#elif`, or `#else` will be included (the `#elif` statement is similar to `else if`). The `defined(name)` expression can be used in `#if` statements. The value of this expression follows these rules: when the name is already defined, its value is 1; otherwise, its value is 0.

To ensure that the contents of the `hdr.h` file are only included once, you can place the content of that file within a conditional statement as follows:
```c
#if !defined(HDR)
#define HDR
/* Content of hdr.h goes here */
#endif
```
The code snippet test the system variable SYSTME first, and determine which header file to use accordingly.

```c
#if SYSTEM == SYSV
    #define HDR "sysv.h"
#elif SYSTEM == BSD
    #define HDR "bsd.h"
#elif SYSTEM == MSDOS
    #define HDR "msdos.h"
#else
    #define HDR "default.h"
#endif
#include HDR
```

There are two special preprocess syntaxes, `#ifdef` and `#ifndef` to check whether a macro has be defined. The `#ifdef` statement is equivalent to `#if defined(name)`, and the `#ifndef` statement is equivalent to `#if !defined(name)`.

```c
#ifndef HDR 
#define EDR 
/* hdr.h goes here */ 
#endif
```

## Chapter 5 - Pointers and Arrays

![pointer and array](CH5-1.png)

A pointer is a variable that **stores the address of a variable**. In the C programming language, pointers are widely used for several reasons. One reason is that pointers are often the **sole means of expressing certain computations**, and another reason is that, compared to other methods, using pointers can often result in **more efficient and compact code**. The relationship between pointers and arrays is very close.

Pointers, much like the `goto` statement, can make programs difficult to understand. If used carelessly, pointers can easily point to the wrong location. However, with careful use, pointers can be employed to write simple and clear programs.

One of the most significant changes introduced by ANSI C is the explicit specification of rules for manipulating pointers. Additionally, ANSI C uses the `void *` type (a pointer to void) instead of `char *` as the type for generic pointers.

### 5.1 Pointers and Addresses

Firstly, let's illustrate how memory is organized through a simple diagram. Typically, machines have a series of consecutively numbered or addressed storage units. These storage units can be manipulated individually or in contiguous groups. Usually, one byte of a machine can store a data type like char, two adjacent bytes can store a short integer, and four adjacent bytes can store a long integer. A pointer is a group of storage units (usually two or four bytes) that can store an address.

The unary operator **&** is used to **obtain the address of an object**. Therefore, the statement `p = &c;` assigns the address of c to the variable p, and we call p a "pointer to" c.

!!! note
    The address operator & can only be applied to objects in memory, namely **variables** and **array elements**. It cannot be applied to expressions, constants, or variables of the register type.

The unary operator **\*** is the **indirection or dereference operator**. When applied to a pointer, it accesses the object pointed to by the pointer.

!!! note
    It's important to note that a pointer can only point to a specific type of object, meaning each pointer must point to a specific data type. (One exception is a pointer to the void type, which can hold a pointer to any type, but it cannot be dereferenced itself.)

Lastly, since pointers are also variables, they can be directly used in the program without always dereferencing them.

### 5.2 Pointers and Function Parameters

As C language passes parameter values to a called function in a **call-by-value manner**, the called function cannot directly modify the values of variables in the calling function.

!!! note
    Pointer parameters allow the called function to access and modify the values of objects in the calling function.

Generally, when defining function parameters, you can use *pointers*, and when calling the function, you can use &. 

### 5.3 Pointers and Arrays

In the C language, the relationship between **pointers and arrays is very close**, so in the following section, we will discuss pointers and arrays together. Any operation that can be done using array subscripts can also be achieved using pointers. Generally, programs written with **pointers execute faster** than those written with array subscripts, but on the other hand, programs implemented with pointers can be **slightly more challenging to understand**.

!!! note
    There is a close correspondence between array subscripts and pointer operations. By definition, the value of a variable or expression of an array type is the address of the 0th element of that array. The array name represents the address of the first element of the array. Therefore, if `pa` is a pointer to an array `a`, the statements `pa = &a[0];` and `pa = a;` are equivalent. References to the array element `a[i]` can also be written as `*(a+i)`.

!!! warning
    It's important to remember that there is a difference between array names and pointers. Pointers are variables, so statements like `pa = a;` and `pa++;` are legal in C. However, array names are not variables, so statements like `a = pa;` and `a++;` are illegal.

When passing an array name to a function, the address of the first element of the array is actually passed. In the called function, this parameter is a local variable; therefore, **array name parameters must be pointers**, which are variables that store address values.

In function definitions, the formal parameters `char s[];` and `char *s;` are equivalent. We usually prefer the latter form because it more intuitively indicates that the parameter is a pointer. If an array name is passed to a function, the function can determine whether to treat it as an array or a pointer, and then operate on the parameter accordingly. To describe functions more intuitively, both array and pointer representations can be used simultaneously in the function.

If you are certain that the corresponding element exists, you can access elements before the first element of an array using subscripts. Expressions like `p[-1]` and `p[-2]` are syntactically legal and refer to the elements two and one positions before `p[0]`, respectively. Of course, referencing objects beyond the array boundaries is illegal.

### 5.4 Address Arithmetic

The method of address arithmetic in the C language is consistent and systematic. Integrating pointer, array, and address arithmetic is a major advantage of this language.

Generally, like other types of variables, pointers can also be initialized. Typically, the meaningful initialization value for a pointer is either 0 or an expression representing an address. For the latter, the address represented by the expression must be of a data type previously defined with the appropriate type.

**Pointers and integers cannot be mutually converted**, except for 0, which is the only exception. The constant 0 can be assigned to a pointer, and pointers can be compared with the constant 0. In programs, the symbolic constant `NULL` is often used instead of the constant 0, making it clear that 0 is a special value for pointers. The symbolic constant `NULL` is defined in the standard header file `<stddef.h>`, and it is frequently used later in this section.

Firstly, in some cases, comparisons can be made between pointers. For example, if pointers `p` and `q` point to members of the same array, they can be subjected to relational comparison operations similar to `==`, `!=`, `<`, and `>=`.

Any comparison between a pointer and 0, either for equality or inequality, is meaningful.

However, arithmetic or comparison operations between pointers pointing to elements of different arrays are undefined. (There is one exception: in pointer arithmetic, the address of the element following the last element of an array can be used.)

**Pointer arithmetic is consistent**: if the data type being processed is a floating-point type that occupies more storage space than a character type, and `p` is a pointer to a floating-point type, then after executing `p++`, `p` will point to the address of the next floating-point number. **All pointer operations automatically take into account the length of the object they point to.**

Valid pointer operations include assignment between pointers of the same type; addition or subtraction operations between pointers and integers; subtraction or comparison operations between two pointers pointing to elements of the same array; and assignment of a pointer to 0 or comparison operations between a pointer and 0. All other forms of pointer operations are illegal, such as addition, multiplication, division, shift, or masking operations between two pointers; addition operations between a pointer and float or double types; and operations that directly assign a pointer to an object of one type to a pointer to an object of another type without explicit type conversion (except when one of the pointers is of type void *).

### 5.5 Character Pointers and Functions

String constants are character arrays. In the internal representation of a string, the character array is terminated by the null character **'\0'**, so programs can find the end of the character array by checking for the null character. As a result, the number of storage units occupied by string constants is one more than the number of characters within double quotes.

The most common usage of string constants is perhaps as function parameters, for example: `printf("hello, world\n");`. When a string like this appears in a program, it is actually accessed through a character pointer. In the above statement, `printf` receives a pointer to the first character of the character array. In other words, **string constants can be accessed through a pointer to their first element**.

Apart from being used as function parameters, string constants have other uses. Assuming the pointer `pmessage` is declared as follows: `char *pmessage;`, the statement `pmessage = "now is the time";` assigns a pointer to this character array to `pmessage`. This process does not involve copying the string but only pointer operations. **C language does not provide an operator to handle the entire string as a whole**.

There is a significant difference between the following two definitions:

```c
char amessage[] = "now is the time"; /* Defines an array */
char *pmessage = "now is the time"; /* Defines a pointer */
```

In the given statement, `amessage` is a one-dimensional array that is only large enough to hold an initialized string and an empty character '\0'. Individual characters in the array can be modified, but `amessage` always points to the same storage location. On the other hand, `pmessage` is a pointer whose initial value points to a string constant. Later, it can be modified to point to other addresses, but attempting to modify the content of the string is undefined (see the diagram below).

![character array vs pointer](arrayvspointer.png)

Standard usage for pushing and popping from the stack.

```c
*p++ = val; /* push */
val = *--p; /* pop */
```

### 5.6 Pointer Arrays and Pointers to Pointers

Since pointers are variables themselves, they can be stored in arrays like other variables.

!!! tip
    In general, it's best to divide a program into several functions that naturally correspond to the problem, controlling the execution of other functions through the main function.

### 5.7 Multidimensional Arrays

C language provides multidimensional arrays similar to matrices, but they are not as widely used as pointer arrays. This section introduces the characteristics of multidimensional arrays.

A two-dimensional array is denoted as `array[i][j]`. Elements are stored by rows, so when accessing the array in storage order, the rightmost array subscript (i.e., column) changes fastest.

Arrays can be initialized using an initial value table enclosed in curly braces, with each row of the two-dimensional array initialized by the corresponding sub-list.

!!! note
    If you pass a two-dimensional array as a parameter to a function, the number of columns in the array must be specified in the function parameter declaration.

### 5.8 Initialization of Pointer Arrays

The initialization syntax for pointer arrays is similar to the initialization syntax for other types of objects discussed earlier.

### 5.9 Pointers and Multidimensional Arrays

For beginners in C, it's easy to confuse the difference between two-dimensional arrays and pointer arrays. Initialization of pointer arrays must be done explicitly, either through static initialization or by code.

An important advantage of pointer arrays is that the length of each row in the array can be different.

!!! note
    Pointer arrays are most commonly used to store strings of different lengths.

### 5.10 Command Line Arguments

In environments supporting the C language, command line arguments can be passed to a program at the start of execution. When calling the main function, it takes two parameters. The value of the first parameter (usually called argc, for argument count) represents the number of arguments in the command line when running the program. The second parameter (called argv, for argument vector) is a pointer to an array of strings, where each string corresponds to an argument. We typically use multi-level pointers to handle these strings.

According to the C language convention, the value of argv[0] is the name of the program that launched it, so argc is at least 1. If argc is 1, it means there are no command line arguments after the program name. In the example above, argc is 3, and the values of argv[0], argv[1], and argv[2] are "echo", "hello,", and "world," respectively. The first optional parameter is argv[1], and the last optional parameter is argv[argc-1]. Additionally, the ANSI standard requires that the value of argv[argc] must be a null pointer.

### 5.11 Pointers to Functions

In the C language, functions themselves are not variables, but pointers to functions can be defined. This type of pointer can be assigned, stored in arrays, passed to functions, and used as a return value.

Note the difference between `int (*comp)(void *, void *)` and `int *comp(void *, void *)`. The former is a pointer to a function, while the latter is a function that returns a pointer to int.

### 5.12 Complex Declarations

C language is often criticized for its syntax regarding declarations, especially when dealing with function pointers. The syntax aims to make declarations and usage consistent. For simple cases, C language practices are effective, but for more complex situations, confusion can arise because C language declarations are not read from left to right and involve many parentheses.

Consider the following two declarations:

```c
int *f(); /* f: function returning pointer to int */
int (*pf)(); /* pf: pointer to function returning int */
```

While complex declarations are rarely used in practice, understanding and, if necessary, using them is important. A good approach to creating complex declarations is to use typedef to synthesize them step by step.