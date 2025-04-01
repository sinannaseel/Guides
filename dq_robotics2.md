# Dual Quaternions Guide

This guide covers the foundations and practical applications of **dual quaternions**, starting from complex numbers, moving through quaternions, and extending to dual quaternion-based modeling and control in robotics. Ideal for researchers, students, and engineers working in robotics and related fields.

---

## Contents
- [Installation and Setup](#installation-and-setup)
- [Matlab Basics](#matlab-basics)
- [Quaternions](#quaternions)
- [Unit Quaternions and Rotations](#unit-quaternions-and-rotations)
- [Dual Numbers](#dual-numbers)
- [Dual Quaternions](#dual-quaternions)
- [Plücker Lines and Planes](#plucker-lines-and-planes)
- [Distance Calculations](#distance-calculations)
- [Robot Control with Dual Quaternions](#robot-control-with-dual-quaternions)
- [Obstacle Avoidance Example](#obstacle-avoidance-example)

---

## Installation and Setup

### CoppeliaSim
1. Download from: [CoppeliaSim V4.1.0](https://www.coppeliarobotics.com/previousVersions)
2. Extract and run using terminal:

```bash
cd Downloads
# extract
tar -xf CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz
cd CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
./coppeliaSim.sh
```

### Matlab & DQ Robotics
1. Sign up for Matlab Onramp.
2. Download `.mltbx` from [DQ Robotics Matlab GitHub](https://github.com/dqrobotics/matlab/releases/tag/20.04.0.1).
3. Upload and install in Matlab Onramp.

---

## Matlab Basics
Matlab uses `.m` for scripts and `.mlx` for live scripts.

```matlab
format long
a1 = 5;
a2 = 67;
add = a1 + a2;
sub = a1 - a2;
mul = a1 * a2;
div = a1 / a2;
```

---

## Quaternions

### Complex Numbers Review
```matlab
include_namespace_dq
c1 = 5 + 28*i_;
i_ * i_;
```

### Quaternion Definition
$$
h = a + b\hat{i} + c\hat{j} + d\hat{k}
$$

### Operations:
- Addition, Subtraction: like complex numbers
- Multiplication: Not commutative
- Conjugate:

$$
(h)^* = a - b\hat{i} - c\hat{j} - d\hat{k}
$$

```matlab
conj(h1)
norm(h1)
vec3(h1)
vec4(h1)
```

---

## Unit Quaternions and Rotations

$$
r = \cos(\phi/2) + \hat{v}\sin(\phi/2)
$$

```matlab
r1 = cos(pi/4) + i_*sin(pi/4);
norm(r1);
```

- Double cover: `r` and `-r` represent same rotation

---

## Dual Numbers

$$
d = a + b\epsilon,\quad \epsilon^2 = 0
$$

```matlab
d1 = 10 - 239*E_;
P(d1); % primary part
D(d1); % dual part
```

---

## Dual Quaternions

$$
h = h_1 + \epsilon h_2\quad h_1, h_2 \in \mathbb{H}
$$

```matlab
h1 = 5 + 6*i_ + 7*j_ + 8*k_ + E_*(9 + 15*i_ + 7*j_ + 8*k_);
```

Operators:
```matlab
norm(h1)
Re(h1)
Im(h1)
P(h1)
D(h1)
vec6(h1)
vec8(h1)
```

---

## Plücker Lines and Planes

### Line:
$$
\underline{l} = l + \epsilon m
$$

```matlab
l1 = i_;
p_l1 = DQ(0);
m1 = cross(p_l1, l1);
l1_dq = l1 + E_*m1;
```

### Plane:
$$
\pi = n_\pi + \epsilon d_\pi
$$

```matlab
n_pi1 = k_;
p_pi1 = DQ(0);
d_pi1 = dot(p_pi1, n_pi1);
pi1 = n_pi1 + E_*d_pi1;
```

---

## Distance Calculations

### Point to Point
```matlab
norm(p1 - p2)
```

### Point to Line
```matlab
norm(cross(p, l1) - m1)
```

### Point to Plane
```matlab
d_p_pi1 = dot(p, n_pi1) - d_pi1
```

### Line to Line
```matlab
d_l1_l2 = sqrt(DQ_Geometry.line_to_line_squared_distance(l1_dq, l2_dq))
```

---

## Robot Control with Dual Quaternions

### 1-DOF Planar Robot

- Forward Kinematics:
```matlab
x_w_1 = cos(theta1/2.0) + k_*sin(theta1/2.0);
x_1_r = 1 + 0.5*E_*i_*l1;
x_w_r = x_w_1 * x_1_r;
t_w_r = translation(x_w_r);
```

- Inverse Kinematics:
```matlab
theta = acos(tx / l1);
```

- Jacobian:
```matlab
j = l1*(-i_*sin(theta1) + j_*cos(theta1));
Jt = vec3(j);
t_dot = Jt * theta1_dot;
```

---

## Obstacle Avoidance Example

Using V-REP and DQ Robotics to simulate obstacle avoidance:

```matlab
translation_controller.set_inequality_constraint(W, w);
u = translation_controller.compute_setpoint_control_signal(q, vec4(td));
q = q + u * tau;
```

For complete simulation see the code section `Obstacle Avoidance` in this repo.

---

## Notes
- All equations follow conventions of DQ Robotics.
- Visual examples and plots are included using the `plot()` function in Matlab.
- Refer to images and figures in `images/` if included in the repo.

---

Made with ❤️ by Naseel Sinan· Based on research & DQ Robotics documentation

