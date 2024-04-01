# C++ Language Study Notes

<div class="grid cards" markdown>

-   :fontawesome-solid-book:{ .lg .middle } __C++ Primer Plus 🎯__

    ---
    Authors: Stephen Prata

    [:octicons-arrow-right-24: <a href="https://zhjwpku.com/assets/pdf/books/C++.Primer.Plus.6th.Edition.Oct.2011.pdf" target="_blank"> Download PDF </a>](#)

</div>

## Table of Contents

- [x] CH01 Getting Started with C++
- [x] CH02 Setting Out to C++
- [x] CH03 Dealing with Data
- [x] CH04 Compound Types
- [ ] CH05 Loops and Relational Expressions
- [ ] CH06 Branching Statements and Logical Operators
- [ ] CH07 Functions: C++’s Programming Modules
- [ ] CH08 Adventures in Functions
- [ ] CH09 Memory Models and Namespaces
- [ ] CH10 Objects and Classes
- [ ] CH11 Working with Classes
- [ ] CH12 Classes and Dynamic Memory Allocation
- [ ] CH13 Class Inheritance
- [ ] CH14 Reusing Code in C++
- [ ] CH15 Friends, Exceptions, and More
- [ ] CH16 The string Class and the Standard Template Library
- [ ] CH17 Input, Output, and Files
- [ ] CH18 Visiting the New C++ Standard

## How this book is organized

This book is organized into 18 chapters and 10 appendices. It introduces the basic concepts and techniques of C++ in detail and in detail through a large number of short and concise programs and devotes a chapter to the new features added by C++11.

- **Chapter 1: Getting Started with C++** - Chapter 1 relates how Bjarne Stroustrup created the C++ programming language by adding object-oriented programming support to the C language. You’ll learn the distinctions between **procedural** languages, such as C, and **object-oriented** languages, such as C++.You’ll read about the joint ANSI/ISO work to develop a C++ standard. This chapter discusses the mechanics of creating a C++ program, outlining the approach for several current C++ compilers. Finally, it describes the conventions used in this book.

- **Chapter 2: Setting Out to C++** — Chapter 2 guides you through the process of creating simple C++ programs. You’ll learn about the role of the main() function and about some of the kinds of statements that C++ programs use. You’ll use the predefined cout and cin objects for program output and input, and you’ll learn about creating and using variables. Finally, you’ll be introduced to functions, C++’s programming modules.

- **Chapter 3: Dealing with Data** — C++ provides built-in types for storing two kinds of data: **integers (numbers with no fractional parts)** and **floating-point numbers (numbers with fractional parts)**. To meet the diverse requirements of programmers, C++ offers several types in each category. Chapter 3 discusses those types, including creating variables and writing constants of various types. You’ ll also learn how C++ handles implicit and explicit conversions from one type to another. 

- **Chapter 4: Compound Types** — C++ lets you construct more elaborate types from the basic built-in types. The most advanced form is the class, discussed in Chapters 9 through 13. Chapter 4 discusses other forms, including **arrays**, which hold several values of a single type; **structures**, which hold several values of unlike types; and **pointers**, which identify locations in memory. You’ll also learn how to create and store text strings and to handle text I/O by using C-style character arrays and the C++ string class. Finally, you’ll learn some of the ways C++ handles memory allocation, including using the new and delete operators for managing memory explicitly.

- **Chapter 5: Loops and Relational Expressions** — Programs often must perform repetitive actions, and C++ provides three looping structures for that purpose: the **for loop**, the **while loop**, and the **do while loop**. Such loops must know when they should terminate, and the C++ relational operators enable you to create tests to guide such loops. In Chapter 5 you learn how to create loops that read and process input character-by-character. Finally, you’ll learn how to create two-dimensional arrays and how to use nested loops to process them. 

- **Chapter 6: Branching Statements and Logical Operators** — Programs can behave intelligently if they can tailor their behavior to circumstances. In Chapter 6 you’ll learn how to control program flow by using the if, if else, and switch statements and the conditional operator. You’ll learn how to use logical operators to help express decision-making tests. Also, you’ll meet the cctype library of functions for evaluating character relations, such as testing whether a character is a digit or a nonprinting character. Finally, you’ll get an introductory view of file I/O.

- **Chapter 7: Functions: C++’s Programming Modules** — Functions are the basic building blocks of C++ programming. Chapter 7 concentrates on features that C++ functions share with C functions. In particular, you’ll review the general for-mat of a function definition and examine how function prototypes increase the reliability of programs. Also, you’ll investigate how to write functions to process arrays, character strings, and structures. Next, you’ll learn about recursion, which is when a function calls itself and see how it can be used to implement a divide-and-conquer strategy. Finally, you’ll meet pointers to functions, which enable you to use a function argument to tell one function to use a second function.

- **Chapter 8: Adventures in Functions** — Chapter 8 explores the new features C++ adds to functions. You’ll learn about **inline functions**, which can speed program execution at the cost of additional program size. You’ll work with **reference variables**, which provide an alternative way to pass information to functions. Default arguments let a function automatically supply values for function arguments that you omit from a function call. **Function overloading** lets you create functions having the same name but taking different argument lists. All these features have frequent use in class design. Also you’ll learn about function templates, which allow you to specify the design of a family of related functions.

- **Chapter 9: Memory Models and Namespaces** — Chapter 9 discusses putting together multifile programs. It examines the choices in allocating memory, looking at different methods of managing memory and at scope, linkage, and namespaces, which determine what parts of a program know about a variable.

- **Chapter 10: Objects and Classes** — A **class** is a user-defined type, and an object (such as a variable) is an **instance** of a class. Chapter 10 introduces you to object-oriented programming and to class design. A class declaration describes the information stored in a class object and also the operations (class methods) allowed for class objects. Some parts of an object are visible to the outside world (the public portion), and some are hidden (the private portion). Special class methods (constructors and destructors) come into play when objects are created and destroyed. You will learn about all this and other class details in this chapter, and you’ll see how classes can be used to implement ADTs, such as a stack.

- **Chapter 11: Working with Classes — I** — Chapter 11 you’ll further your under-standing of classes. First, you’ll learn about **operator overloading**, which lets you define how operators such as + will work with class objects. You’ll learn about **friend functions**, which can access class data that’s inaccessible to the world at large. You’ll see how certain constructors and overloaded operator member functions can be used to manage conversion to and from class types. 

- **Chapter 12: Classes and Dynamic Memory Allocation** — Often it’s useful to have a class member point to dynamically allocated memory. **If you use new in a class constructor to allocate dynamic memory, you incur the responsibilities of providing an appropriate destructor, of defining an explicit copy constructor, and of defining an explicit assignment operator.** Chapter 12 shows you how and discusses the behavior of the member functions generated implicitly if you fail to provide explicit definitions. You’ll also expand your experience with classes by using pointers to objects and studying a queue simulation problem.

- **Chapter 13: Class Inheritance** — One of the most powerful features of object-oriented programming is **inheritance**, by which a derived class inherits the features of a base class, enabling you to reuse the base class code. Chapter 13 discusses public inheritance, which models `is-a` relationship, meaning that a derived object is a special case of a base object. For example, a physicist is a special case of a scientist. Some inheritance relationships are **polymorphic**, meaning you can write code using a mixture of related classes for which the same method name may invoke behavior that depends on the object type. Implementing this kind of behavior necessitates using a new kind of member function called a **virtual function**. Sometimes using abstract base classes is the best approach to inheritance relationships. This chapter discusses these matters, pointing out when public inheritance is appropriate and when it is not.

- **Chapter 14: Reusing Code in C++** — Public inheritance is just one way to reuse code. Chapter 14 looks at several other ways. Containment is when one class contains members that are objects of another class. It can be used to model has a relationship, in which one class has components of another class. For example, an automobile has a motor. You also can use private and protected inheritance to model such relationships. This chapter shows you how and points out the differences among the different approaches. Also, you’ll learn about class templates, which let you define a class in terms of some unspecified generic type, and then use the template to create specific classes in terms of specific types. For example, a stack template enables you to create a stack of integers or a stack of strings. Finally, you’ll learn about multiple public inheritance, whereby a class can derive from more than one class.

- **Chapter 15: Friends, Exceptions, and More** — Chapter 15 extends the discussion of friends to include friend classes and friend member functions. Then it presents several new developments in C++, beginning with exceptions, which provide a mechanism for dealing with unusual program occurrences, such an inappropriate function argument values and running out of memory. Then you’ll learn about RTTI, a mechanism for identifying object types. Finally, you’ll learn about the safer alternatives to unrestricted typecasting.

- **Chapter 16: The string Class and the Standard Template Library** — Chapter 16 discusses some useful class libraries recently added to the language. The string class is a convenient and powerful alternative to traditional C-style strings. The auto_ptr class helps manage dynamically allocated memory. The STL provides several generic containers, including template representations of arrays, queues, lists, sets, and maps. It also provides an efficient library of generic algorithms that can be used with STL containers and also with ordinary arrays. The valarray template class provides sup-port for numeric arrays.

- **Chapter 17: Input, Output, and Files** — Chapter 17 reviews C++ I/O and dis-cusses how to format output. You’ll learn how to use class methods to determine the state of an input or output stream and to see, for example, whether there has been a type mismatch on input or whether the end-of-file has been detected. C++ uses inheritance to derive classes for managing file input and output. You’ll learn how to open files for input and output, how to append data to a file, how to use binary files, and how to get random access to a file. Finally, you’ll learn how to apply standard I/O methods to read from and write to strings.

- **Chapter 18: Visiting with the New C++ Standard** — Chapter 18 begins by reviewing several C++11 features introduced in earlier chapters, including new types, uniform initialization syntax, automatic type deduction, new smart pointers, and scoped enumerations. The chapter then discusses the new rvalue reference type and how it’s used to implement a new feature called move semantics. Next, the chapter covers new class features, lambda expressions, and variadic templates. Finally, the chapter outlines many new features not covered in earlier chapters of the book.

- Appendix A: Number Bases—Appendix A discusses octal, hexadecimal, and
binary numbers.
- Appendix B: C++ Reserved Words—Appendix B lists C++ keywords.
- Appendix C: The ASCII Character Set—Appendix C lists the ASCII character
set, along with decimal, octal, hexadecimal, and binary representations.
- Appendix D: Operator Precedence—Appendix D lists the C++ operators in
order of decreasing precedence.
- Appendix E: Other Operators—Appendix E summarizes the C++ operators,
such as the bitwise operators, not covered in the main body of the text.
- Appendix F: The string Template Class—Appendix F summarizes string
class methods and functions.
- Appendix G: The Standard Template Library Methods and Functions—
Appendix G summarizes the STL container methods and the general STL 
algorithm functions.
- Appendix H: Selected Readings and Internet Resources—Appendix H lists
some books that can further your understanding of C++.
- Appendix I: Converting to ISO Standard C++—Appendix I provides guidelines
for moving from C and older C++ implementations to ANSI/ISO C++.
- Appendix J: Answers to Chapter Review—Appendix J contains the answers to
the review questions posed at the end of each chapter.

