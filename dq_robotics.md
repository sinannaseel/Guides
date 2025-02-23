Below is a concise Markdown guide covering dual quaternions and robot control in MATLAB. You can copy this into a file (e.g., `README.md`) for your Git repository.

---

```markdown
# Dual Quaternions & Robot Control in MATLAB

This guide briefly summarizes key concepts and operations using MATLAB for quaternions, dual quaternions, and a simple 1-DoF planar robot control. (Requires the DQ Robotics toolbox.)

---

## MATLAB Essentials

- **Precision Display:**  
  ```matlab
  format long
  ```

- **Basic Arithmetic:**
  ```matlab
  a1 = 5; a2 = 67;
  add_0 = a1 + a2;
  ```

---

## Quaternion Fundamentals

- **Representation:**  
  $$ h = a + b\,\hat{i} + c\,\hat{j} + d\,\hat{k} $$

- **Key Operations:**  
  ```matlab
  h_conj = conj(h);
  h_norm = norm(h);
  v = vec3(h);      % Imaginary part as R^3 vector
  v4 = vec4(h);     % Full quaternion as R^4 vector
  ```

- **Hamilton Operators:**  
  ```matlab
  vec4(h1 * h2) == hamiplus4(h1) * vec4(h2);
  ```

---

## Unit Quaternions for Rotations

- **Definition:**  
  $$ r = \cos(\phi/2) + v\,\sin(\phi/2) $$  
  *Example – Rotation about x-axis:*
  ```matlab
  r1 = cos(pi/4) + i_*sin(pi/4);
  norm(r1)  % Should equal 1
  ```
- **Double Cover:** Both `r1` and `-r1` represent the same rotation.

---

## Dual Quaternions

- **Structure:**  
  $$ x = h_1 + h_2\,\epsilon,\quad \epsilon^2 = 0 $$
- **Pose Transformation:**  
  $$ x = r + \frac{1}{2}\,\epsilon\,t\,r $$
  *Example:*
  ```matlab
  t1 = i_;
  r1 = cos(pi/16) + k_*sin(pi/16);
  x1 = r1 + 0.5*E_*t1*r1;
  ```
- **Useful Functions:**  
  ```matlab
  primary = P(x1);
  dual    = D(x1);
  ```

---

## Plücker Lines & Planes

- **Plücker Line:**  
  $$ \underline{l} = l + \epsilon m $$
  *Example for x-axis:*
  ```matlab
  l1 = i_;
  p_l1 = DQ(0);
  m1 = cross(p_l1, l1);
  l1_dq = l1 + E_*m1;
  % Check:
  is_pure(l1_dq)
  is_unit(l1_dq)
  ```

- **Plane Representation:**  
  $$ \underline{\pi} = n_\pi + \epsilon d_\pi $$
  *Example – x-y plane:*
  ```matlab
  n_pi1 = k_;
  p_pi1 = DQ(0);
  d_pi1 = dot(p_pi1, n_pi1);
  pi1 = n_pi1 + E_*d_pi1;
  % Verify:
  is_unit(pi1)
  is_plane(pi1)
  ```

---

## 1-DoF Planar Robot Control

### Class Definition

```matlab
classdef OneDofPlanarRobot
    properties (Access = private)
        l1  % Robot length (m)
    end
    methods
        function obj = OneDofPlanarRobot(l)
            obj.l1 = l;
        end
        function t_w_r = fkm(obj, theta1)
            include_namespace_dq
            % Rotation about joint:
            x_w_1 = cos(theta1/2) + k_*sin(theta1/2);
            % Translation along link:
            x_1_r = 1 + 0.5*E_*i_*obj.l1;
            % Combined pose:
            t_w_r = translation(x_w_1 * x_1_r);
        end
        function theta = ikm_tx(obj, tx)
            theta = acos(tx / obj.l1);
        end
        function Jt = translation_jacobian(obj, theta1)
            include_namespace_dq
            j = obj.l1 * (-i_*sin(theta1) + j_*cos(theta1));
            Jt = vec3(j);
        end
    end
end
```

### Usage Examples

- **Forward Kinematics & Plotting:**
  ```matlab
  l1 = 1;
  robot = OneDofPlanarRobot(l1);
  theta1 = 0.9;
  t_w_r = robot.fkm(theta1)
  robot.plot(theta1);
  ```

- **Inverse Kinematics (x-axis):**
  ```matlab
  tx = -0.47;
  theta1 = robot.ikm_tx(tx);
  robot.plot(theta1);
  ```

- **Differential Kinematics:**
  - *End-effector velocity:*
    ```matlab
    theta1_dot = 0.14;
    Jt = robot.translation_jacobian(theta1);
    t_dot = Jt * theta1_dot;
    ```
  - *Joint velocity from end-effector velocity:*
    ```matlab
    t_dot = DQ([vx, vy, vz]);
    theta_dot = pinv(Jt) * vec3(t_dot);
    ```

---
