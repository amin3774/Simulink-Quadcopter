# Quadcopter Modeling and Control in MATLAB/Simulink

A MATLAB/Simulink project for modeling a quadcopter's six-degree-of-freedom (6-DOF) dynamics and designing and comparing feedback controllers. The project starts with a **nonlinear rigid-body model**, derives a **linear state-space model around hover**, and investigates **PID**, **Linear Quadratic Regulator (LQR)**, and **Model Predictive Control (MPC)** approaches.

> **Project status:** Simulation and controller development. Hardware deployment is a future goal; results in this repository should not be interpreted as flight-test results.

## 1. Objectives

- Develop a nonlinear 6-DOF quadcopter plant in Simulink.
- Derive and validate a linearized model at the hover equilibrium.
- Design and test position/attitude controllers, including PID, LQR, and MPC.
- Compare tracking behavior, trajectories, and control signals under consistent simulation conditions.
- Prepare the control architecture for eventual embedded implementation.

## 2. Model parameters

The nominal parameters used in the simulation are:

| Parameter | Symbol | Value | Unit |
|---|---|---:|---|
| Mass | $m$ | 0.5 | kg |
| Arm length | $l$ | 0.2 | m |
| Gravitational acceleration | $g$ | 9.8 | m/s² |
| Roll moment of inertia | $I_x$ | 0.005 | kg·m² |
| Pitch moment of inertia | $I_y$ | 0.005 | kg·m² |
| Yaw moment of inertia | $I_z$ | 0.009 | kg·m² |
| Rotor thrust coefficient | $k_f$ | $10^{-5}$ | N/(rad/s)² |
| Rotor reaction-torque coefficient | $k_m$ | $10^{-7}$ | N·m/(rad/s)² |

The inertia matrix is $I=\mathrm{diag}(I_x,I_y,I_z)$. Rotor coefficients assume rotor speed in rad/s; confirm the units if your implementation uses another convention.

## 3. Nonlinear quadcopter model

The quadcopter is modeled as a rigid body with translational and rotational motion. A 12-state representation is

$$
\mathbf{x} =
\begin{bmatrix}
x & y & z & \dot x & \dot y & \dot z & \phi & \theta & \psi & p & q & r
\end{bmatrix}^{\mathsf T},
$$

where $(x,y,z)$ is inertial-frame position, $(\phi,\theta,\psi)$ are roll/pitch/yaw Euler angles, and $(p,q,r)$ are angular rates expressed in the body frame.

The control-input vector is

$$
\mathbf{u} = \begin{bmatrix}T & \tau_x & \tau_y & \tau_z\end{bmatrix}^{\mathsf T},
$$

where $T$ is the total thrust and $\boldsymbol{\tau}$ is the commanded body torque. With an inertial $z$-axis pointing upward and thrust along the positive body $z$-axis, the nonlinear equations are

$$
\dot{\mathbf{p}} = \mathbf{v}, \qquad
m\dot{\mathbf{v}} = T R(\phi,\theta,\psi)\mathbf{e}_3 - mg\mathbf{e}_3,
$$

$$
\dot{\boldsymbol{\eta}} = W(\phi,\theta)\boldsymbol{\omega}, \qquad
I\dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega}\times(I\boldsymbol{\omega}),
$$

with $\boldsymbol{\eta}=[\phi,\theta,\psi]^{\mathsf T}$, $\boldsymbol{\omega}=[p,q,r]^{\mathsf T}$, $\mathbf e_3=[0,0,1]^{\mathsf T}$, and $R=R_z(\psi)R_y(\theta)R_x(\phi)$. The Euler-rate mapping (for ZYX Euler angles) is

$$
W(\phi,\theta)=
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta\\
0 & \cos\phi & -\sin\phi\\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}.
$$

This representation has an Euler-angle singularity at $\theta=\pm 90^\circ$; the study focuses on near-hover operation.

For individual rotor speeds $\omega_i$, a simple actuator model is $F_i=k_f\omega_i^2$ and $M_i=k_m\omega_i^2$. The conversion from the four rotor forces to $T$ and $\boldsymbol{\tau}$ depends on the rotor numbering, rotation directions, and **+ / ×** frame configuration used in the Simulink model.

## 4. Linearization around hover

The hover equilibrium is defined by zero translational velocity, level attitude, and zero angular velocity:

$$
\phi_0=\theta_0=0, \quad \mathbf v_0=\mathbf0, \quad
\boldsymbol\omega_0=\mathbf0, \quad T_0=mg=4.9\ \mathrm{N}.
$$

The yaw angle and position can be chosen as zero for the nominal linearization. Using deviations $\delta\mathbf x=\mathbf x-\mathbf x_0$ and $\delta\mathbf u=\mathbf u-\mathbf u_0$, the local model is

$$
\delta\dot{\mathbf{x}} = A\,\delta\mathbf{x} + B\,\delta\mathbf{u}.
$$

For the state ordering above and nominal yaw $\psi_0=0$, the nonzero relationships are

$$
\begin{aligned}
\delta\dot{x}&=\delta v_x,& \delta\dot v_x&=g\,\delta\theta,\\
\delta\dot{y}&=\delta v_y,& \delta\dot v_y&=-g\,\delta\phi,\\
\delta\dot{z}&=\delta v_z,& \delta\dot v_z&=\delta T/m,\\
\delta\dot\phi&=\delta p,& \delta\dot p&=\delta\tau_x/I_x,\\
\delta\dot\theta&=\delta q,& \delta\dot q&=\delta\tau_y/I_y,\\
\delta\dot\psi&=\delta r,& \delta\dot r&=\delta\tau_z/I_z.
\end{aligned}
$$

Thus the nonzero entries of $A$ are $A_{1,4}=A_{2,5}=A_{3,6}=A_{7,10}=A_{8,11}=A_{9,12}=1$, $A_{4,8}=g$, and $A_{5,7}=-g$ (1-based indexing). The input matrix has $B_{6,1}=1/m=2$, $B_{10,2}=1/I_x=200$, $B_{11,3}=1/I_y=200$, and $B_{12,4}=1/I_z\approx111.11$.

**Important:** The linearized model is intended for small disturbances near hover. Controller performance at large tilt angles or aggressive trajectories must be evaluated on the nonlinear plant, rather than inferred from the linear approximation alone.

## 5. Controllers

### 5.1 PID / PI control

A baseline proportional-integral (PI) controller is used for altitude tracking. The general continuous-time PI control law is

$$
u_z(t)=K_p\,e_z(t)+K_i\int_0^t e_z(\sigma)\,d\sigma,
\qquad e_z=z_{\mathrm{ref}}-z.
$$

Here $\nu_z$ denotes a thrust **correction** about hover; the complete thrust command is $T=mg+\nu_z$. The PID/PI approach is also useful for studying the challenges of coupling lateral position control to roll and pitch.

### 5.2 Linear Quadratic Regulator (LQR)

LQR computes a state-feedback gain $K$ by minimizing the quadratic cost

$$
J=\int_0^\infty \left(\delta\mathbf{x}^{\mathsf T}Q\delta\mathbf{x}+\delta\mathbf{u}^{\mathsf T}R\delta\mathbf{u}\right)dt,
$$

with $Q\succeq0$ and $R\succ0$. The regulation law is

$$
\delta\mathbf{u}=-K\delta\mathbf{x}.
$$

For nonzero position references, the implementation must introduce an appropriate reference equilibrium or tracking architecture; simply substituting an arbitrary reference into the regulation law is not a general solution. The resulting torque and thrust commands are applied to the nonlinear quadcopter model for evaluation.

### 5.3 Model Predictive Control (MPC)

MPC uses a discrete-time prediction model and optimizes a sequence of future control inputs over a finite prediction horizon. A representative formulation is

$$
\min_{\{\delta\mathbf{u}_{k+i}\}}
\sum_{i=1}^{N_p}\|\mathbf{x}_{k+i}-\mathbf{x}_{\mathrm{ref},k+i}\|_Q^2
+\sum_{i=0}^{N_c-1}\|\delta\mathbf{u}_{k+i}\|_R^2,
$$

subject to the discrete-time dynamics and any modeled control/state constraints. Only the first optimized input is applied before the optimization is repeated at the next sample.

During development, prediction horizons of **30** and **15** steps were investigated. Tracking and trajectory shape are compared with LQR under similar initial conditions and references. The specific weighting matrices, sampling time, and constraints should be documented alongside the implementation to make results reproducible.

## 6. Simulation and comparison

Recommended experiments include:

1. **Hover stabilization:** recovery from small position and attitude perturbations.
2. **Altitude step:** reference tracking in $z$.
3. **3D position tracking:** compare responses to the same $(x,y,z)$ reference.
4. **Controller comparison:** evaluate LQR and MPC with matched initial states and setpoints.
5. **Control feasibility:** monitor total thrust, torques, actuator limits, and tilt angles.

Useful comparison metrics include settling time, overshoot, steady-state error, control effort, and computational cost. Plots can show position versus time and the 3D trajectory with direction arrows.

> Add actual simulation plots, numerical results, and model screenshots here once they are committed to the repository. A visual difference between LQR and MPC trajectories does not by itself establish which controller is better.

## 7. Repository structure

Organize files as appropriate for the current implementation; the following is a **suggested** structure, not a claim about existing files:

```text
quadcopter-control/
├── README.md
├── models/          # Simulink plant and controller models (.slx)
├── scripts/         # Initialization, linearization, controller design (.m)
├── results/         # Exported plots and comparisons
└── docs/            # Derivations and supplementary explanations
```

## 8. Getting started

1. Install MATLAB and Simulink.
2. Open the main `.slx` model in `models/` (replace this path with the actual model location).
3. Run the project initialization script, if present, to load mass, inertia, gravity, and controller parameters.
4. Choose the controller and simulation reference signals.
5. Run the simulation and inspect position, attitude, and control outputs.

**Toolbox requirements:** Update this section with the MATLAB release and installed toolboxes used for your implementation. In particular, state whether MPC is implemented manually or with Model Predictive Control Toolbox.

## 9. Future work

- Compare controller behavior under model uncertainty and external disturbances.
- Include explicit thrust, torque, and motor-speed constraints where applicable.
- Improve trajectory tracking and computational efficiency.
- Investigate deployment to a small embedded quadcopter platform, with hardware-in-the-loop and flight testing as separate validation stages.

## License

No license has been specified yet. Add a `LICENSE` file if you want to grant others permission to use or modify this project.
