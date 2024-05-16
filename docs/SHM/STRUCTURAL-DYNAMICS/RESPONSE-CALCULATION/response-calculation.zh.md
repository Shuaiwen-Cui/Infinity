# 结构动力响应计算

## Newmark-Beta 方法求解动力响应

### 原理

Newmark-beta 方法是一种数值积分方法，用于求解动力学方程，特别适用于结构动力学中的时程分析。它的基本思想是通过在每个时间步上应用加速度、速度和位移的预测公式，逐步求解系统的响应。

### 动力学方程

对于一个多自由度系统，其运动方程可以写成矩阵形式：

\[ M \ddot{u}(t) + C \dot{u}(t) + K u(t) = F(t) \]

其中：
- \(M\) 是质量矩阵
- \(C\) 是阻尼矩阵
- \(K\) 是刚度矩阵
- \(u(t)\) 是位移向量
- \(\dot{u}(t)\) 是速度向量
- \(\ddot{u}(t)\) 是加速度向量
- \(F(t)\) 是外力向量

### Newmark-beta 方法

Newmark-beta 方法使用以下公式来更新每个时间步的位移和速度：

1. 速度更新公式：

\[ \dot{u}_{n+1} = \dot{u}_n + (1 - \gamma) \Delta t \ddot{u}_n + \gamma \Delta t \ddot{u}_{n+1} \]

2. 位移更新公式：

\[ u_{n+1} = u_n + \Delta t \dot{u}_n + \left( \frac{1}{2} - \beta \right) \Delta t^2 \ddot{u}_n + \beta \Delta t^2 \ddot{u}_{n+1} \]

其中：
- \(\Delta t\) 是时间步长
- \(\beta\) 和 \(\gamma\) 是 Newmark 参数，通常选择 \(\beta = 0.25\) 和 \(\gamma = 0.5\)，即所谓的平均加速度法

### 过程

1. **初始条件**：给定初始位移 \(u_0\) 和初始速度 \(\dot{u}_0\)，计算初始加速度 \(\ddot{u}_0\)。

2. **有效刚度矩阵**：

\[ K_{\text{eff}} = K + \frac{\gamma}{\beta \Delta t} C + \frac{1}{\beta \Delta t^2} M \]

3. **时间步推进**：

- 计算有效荷载：

\[ F_{\text{eff}} = F_{n+1} + M \left( \frac{1}{\beta \Delta t^2} u_n + \frac{1}{\beta \Delta t} \dot{u}_n + \left( \frac{1}{2 \beta} - 1 \right) \ddot{u}_n \right) + C \left( \frac{\gamma}{\beta \Delta t} u_n + \left( \frac{\gamma}{\beta} - 1 \right) \dot{u}_n + \Delta t \left( \frac{\gamma}{2 \beta} - 1 \right) \ddot{u}_n \right) \]

- 解方程得到新的位移：

\[ u_{n+1} = K_{\text{eff}}^{-1} F_{\text{eff}} \]

- 计算新的加速度：

\[ \ddot{u}_{n+1} = \frac{1}{\beta \Delta t^2} (u_{n+1} - u_n) - \frac{1}{\beta \Delta t} \dot{u}_n - \left( \frac{1}{2 \beta} - 1 \right) \ddot{u}_n \]

- 计算新的速度：

\[ \dot{u}_{n+1} = \dot{u}_n + \Delta t \left( (1 - \gamma) \ddot{u}_n + \gamma \ddot{u}_{n+1} \right) \]
