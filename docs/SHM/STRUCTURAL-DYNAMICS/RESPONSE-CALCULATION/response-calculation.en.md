# STRUCTURAL RESPONSE CALCULATION

## Newmark-Beta Method for Dynamic Response Calculation

### Principle

The Newmark-beta method is a numerical integration method used to solve dynamic equations, especially for time history analysis in structural dynamics. The basic idea is to iteratively solve for the system's response by applying predictor-corrector formulas for acceleration, velocity, and displacement at each time step.

### Dynamic Equation

For a multi-degree-of-freedom (MDOF) system, the equation of motion can be written in matrix form as:

\[ M \ddot{u}(t) + C \dot{u}(t) + K u(t) = F(t) \]

where:
- \(M\) is the mass matrix
- \(C\) is the damping matrix
- \(K\) is the stiffness matrix
- \(u(t)\) is the displacement vector
- \(\dot{u}(t)\) is the velocity vector
- \(\ddot{u}(t)\) is the acceleration vector
- \(F(t)\) is the external force vector

## Newmark-beta Method

The Newmark-beta method uses the following formulas to update the displacement and velocity at each time step:

1. **Velocity update formula**:

\[ \dot{u}_{n+1} = \dot{u}_n + (1 - \gamma) \Delta t \ddot{u}_n + \gamma \Delta t \ddot{u}_{n+1} \]

2. **Displacement update formula**:

\[ u_{n+1} = u_n + \Delta t \dot{u}_n + \left( \frac{1}{2} - \beta \right) \Delta t^2 \ddot{u}_n + \beta \Delta t^2 \ddot{u}_{n+1} \]

where:
- \(\Delta t\) is the time step size
- \(\beta\) and \(\gamma\) are Newmark parameters, typically \(\beta = 0.25\) and \(\gamma = 0.5\), which correspond to the average acceleration method

## Procedure

1. **Initial Conditions**: Given initial displacement \(u_0\) and initial velocity \(\dot{u}_0\), compute the initial acceleration \(\ddot{u}_0\).

2. **Effective Stiffness Matrix**:

\[ K_{\text{eff}} = K + \frac{\gamma}{\beta \Delta t} C + \frac{1}{\beta \Delta t^2} M \]

3. **Time Stepping**:

- Compute the effective force:

\[ F_{\text{eff}} = F_{n+1} + M \left( \frac{1}{\beta \Delta t^2} u_n + \frac{1}{\beta \Delta t} \dot{u}_n + \left( \frac{1}{2 \beta} - 1 \right) \ddot{u}_n \right) + C \left( \frac{\gamma}{\beta \Delta t} u_n + \left( \frac{\gamma}{\beta} - 1 \right) \dot{u}_n + \Delta t \left( \frac{\gamma}{2 \beta} - 1 \right) \ddot{u}_n \right) \]

- Solve for the new displacement:

\[ u_{n+1} = K_{\text{eff}}^{-1} F_{\text{eff}} \]

- Compute the new acceleration:

\[ \ddot{u}_{n+1} = \frac{1}{\beta \Delta t^2} (u_{n+1} - u_n) - \frac{1}{\beta \Delta t} \dot{u}_n - \left( \frac{1}{2 \beta} - 1 \right) \ddot{u}_n \]

- Compute the new velocity:

\[ \dot{u}_{n+1} = \dot{u}_n + \Delta t \left( (1 - \gamma) \ddot{u}_n + \gamma \ddot{u}_{n+1} \right) \]


