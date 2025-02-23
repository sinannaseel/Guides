# Matlab & Dual Quaternions Tutorial

This guide provides an overview of MATLAB basics, quaternions, dual quaternions, and a simple robot control example using MATLAB. The topics covered include:

- [Basics in MATLAB](#basics-in-matlab)
- [Quaternion Basics](#quaternion-basics)
- [Dual Quaternions Part I](#dual-quaternions-part-i)
- [Dual Quaternions Part II](#dual-quaternions-part-ii)
- [Basic Robot Control Part I](#basic-robot-control-part-i)

---

## Basics in MATLAB

### MATLAB Scripts vs. Live Scripts
- **`.m` files**: Standard MATLAB scripts.
- **`.mlx` files**: MATLAB Live Scripts that combine code, formatted text, and graphics.

### Example Code

```matlab
format long 
% Show 15 significant digits when displaying numbers

a1 = 5;
a2 = 67   % Without semicolon, the result is printed (a2 = 67)

add_0 = a1 + a2
sub_0 = a1 - a2
multiply_0 = a1 * a2
divide_0 = a1 / a2
power2 = a1^2
