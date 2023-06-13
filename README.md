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

https://github.com/komxun/PLOS-Algorithm/blob/05bcc621a2c89750954fc5bbcab71b15181f3c5a/PLOS_Straight.m#L44-L100

![image](https://github.com/komxun/PLOS-Algorithm/assets/133139057/60d935f2-9cfb-4c8d-8779-7a0daaef6dec)


# Result - PLOS Straight-Line
|![PLOS_1line](https://github.com/komxun/PLOS-Algorithm/assets/133139057/7d10f785-53d0-4b31-afc2-f26a28c886c6)|
|:--:|
|PLOS for straight-line path following with $k_1 = 5$ and $k_2 = 0.2$|

# Tuning PLOS 
The PLOS guidance law is stable for $k_1 >0$ and $k_2>0$. With a constant $k_2$, smaller value of $k_1$ oscillates the path along 
the desired straight-line path. The increase in $k_1$ means the pursuit-guidance has more weight, thus, the path will have a less oscillation.

For a constant $k_1$, a smaller value of $k_2$ makes the pursuit guidance more dominant which make the path converges toward the waypoint faster. With a higher value of $k_2$ the vehicles will try to move toward the desired path with a longer converging time. Nonetheless, when $k_2$ becomes too large, the LOS guidance will dominate the pursuit guidance, resulting in an oscillation of the path.

|![PLOS_multiline](https://github.com/komxun/PLOS-Algorithm/assets/133139057/1bb96bec-853d-4cd2-8f33-329cfe0e7050)|
|:--:|
|PLOS for straight-line path following with $k_2 = 0.1$ and various $k_1$|

|![PLOS_multiline_variousk2](https://github.com/komxun/PLOS-Algorithm/assets/133139057/30e8dfe8-5ce1-4a6b-96cb-9b50260daf1a)|
|:--:|
|PLOS for straight-line path following with $k_1 = 50$ and various $k_2$|



