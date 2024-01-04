# C Language Study Notes

## Table of Contents

CH1 - A Tutorial Introduction
CH2 - Types, Operators, and Expressions
CH3 - Control Flow
CH4 - Functions and Program Structure
CH5 - Pointers and Arrays
CH6 - Structures
CH7 - Input and Output
CH8 - The UNIX System Interface

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
