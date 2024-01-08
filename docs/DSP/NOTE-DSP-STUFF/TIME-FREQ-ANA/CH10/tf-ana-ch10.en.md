# Supplement to Some Common Concepts in Time-Frequency Domain Analysis (Linear Time-Invariant Systems, Convolution, Impulse Response, Window Functions, etc.)

## Additivity and Homogeneity

Since these two concepts are two conditions that must be satisfied by linear systems, let's discuss them together. These concepts are easy to understand and align with common everyday knowledge.

Using a somewhat inappropriate analogy, it's like "equivalent exchange."

Buying one candy for one cent and buying ten candies for ten cents is additivity. That is, \( f(ax) = af(x) \).

Buying one candy for one cent, exchanging one seashell for one gold coin, and using one cent and one seashell to get one candy and one gold coin is homogeneity. That is, \( f(x+y) = f(x) + f(y) \).

Systems that satisfy additivity and homogeneity are linear systems. These systems are straightforward; it's easy to compute and understand what output they produce given any input.

## Time-Invariance

Linear systems are straightforward, but do they change? Will you get only five candies instead of ten when you give ten cents one day? Some do change, but some don't. For those resolute systems that don't change with time, we call them linear time-invariant systems.

## Convolution

The continuous definition of convolution is:

\[ y(t) = \int_{-\infty}^{\infty} x(\tau)h(t-\tau)d\tau \]

Convolution is applicable only to linear time-invariant systems.

Understanding the concept of convolution, there are some important features to remember:

**Property 1: Convolution in the time domain is equivalent to multiplication in the frequency domain, and vice versa.**

An example:

![bpd](bpd.png)

In the figure, (a) shows the modulation (multiplication) of high-frequency and low-frequency signals in the time domain, and (b) is the result of convolution in the frequency domain.

The formation of sidebands also verifies the second property of convolution:

**Property 2: Convolution with an impulse function produces a mirrored waveform at each pulse position.**

![illu2](illu2.png)

## Impulse Response

The zero-state response caused by the excitation of a unit impulse function is called the "impulse response" of the system.

In a figurative sense, the impulse response is like the "swelling" after being slapped (unit impulse) in the example above. The shape and duration of the "swelling" differ for different individuals (systems) when subjected to the same intensity of slap (unit impulse). However, for the same individual (linear time-invariant system), each "swelling" is the same. Therefore, as long as you know when and how hard the slap occurred (signal input), you can determine what shape the face will swell into through convolution (system output).

So, in the time domain, knowing the impulse response allows you to calculate the output based on the input. This simplifies the calculation of complex output results into two simple steps:

- 1. Calculate the system's impulse response.
- 2. Convolve the input with the impulse response.

If you pay attention, you'll notice that the above process is performed in the time domain, and the convolution in the time domain is equivalent to multiplication in the frequency domain. Therefore, **the frequency domain transformation of the impulse response = frequency domain transformation of the output ÷ frequency domain transformation of the input = transfer function**. Yes, **the frequency domain transformation of the impulse response is the transfer function of the system in the frequency domain.**

To wrap it up, consider this phrase:

Any linear time-invariant system can be viewed as a digital filter.

We haven't discussed the concept of a digital filter yet, but students in this column should be familiar with it. According to the description above, **a linear time-invariant system can be abstracted into an impulse response in the time domain and a transfer function in the frequency domain**. From the perspective of frequency domain analysis, **after a signal passes through a linear system, the spectrum of the output signal will be the product of the spectrum of the input signal and the system's transfer function**. Certain frequency components in the transfer function have larger magnitudes. Therefore, these frequency components in the original signal will be enhanced, while other frequency components where the magnitudes are small or even zero will be attenuated or eliminated. Hence, **the system's effect is equivalent to weighting the spectrum of the input signal**. For any linear time-invariant system, although the frequency components that are enhanced or attenuated may differ, they serve the same function of frequency filtering.
