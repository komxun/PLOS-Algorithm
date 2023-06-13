# PLOS-Algorithm
A path following algorithm based on Pure pursuit and LOS-based guidance laws

PLOS algorithm is the path following algorithm by the combination of the Pure pursuit guidance law + LOS-based guidance laws

- The Pure pursuit guidance law can drives the UAV to the way point $W_{i+1}$
- The LOS guidance law allows the UAV to steers toward the line-of-sight (LOS)

## The pure pursuit guidance law
```math
\dot{\psi}_p = k_1(\theta_d-\psi)
```
## The LOS guidance law
$$\eqalign{
\dot{\psi}_i &= k_2d \\
d &= R_u\sin(\theta_u-\theta) \approx R_u(\theta_u - theta)
}$$

## The combined guidance (PLOS) law
$$
u = v_a(\dot{\phi}_i + \dot{\psi}_i) = v_a\left(k_1(\theta_d-\psi)+k_2d\right)
$$

# PLOS algorithm for straight-line following
The PLOS algorithm can be summarized below. This has been coded in the main while loop in **PLOS_Straight.m**
![image](https://github.com/komxun/PLOS-Algorithm/assets/133139057/bd31cd54-e452-4fe1-8548-c1607337baa6)


# Result - PLOS Straight-Line

