# farol2_sim

`farol2_sim` provides a simple marine vehicle simulator for the Farol2 stack.
It combines Fossen-style marine craft dynamics with rigid-body kinematics,
thruster and rudder models, and simulated sensor measurements. It supports
six-degree-of-freedom motion as well as models with selected degrees of freedom.
Vehicle properties, actuator geometry, sensors, and environmental conditions are
configurable.

The package focuses on vehicle motion and measurements: it has no collision
simulation or user interface. Water current is represented as a prescribed,
uniform flow, and hydrodynamic effects are described by lumped model coefficients.

## Theory

### Kinematics and water current

Let $\mathcal{I}$ denote the North–East–Down (NED) inertial frame and
$\mathcal{B}$ the body frame, whose axes point forward, starboard, and down. The vehicle pose is described by
the body-origin position $\boldsymbol{p}^{\mathcal{I}}$ and the rotation matrix
$R \in SO(3)$, which maps body-frame vectors into the inertial frame.
The velocity state is

$$
\boldsymbol{\nu}_r =
\begin{bmatrix}
\boldsymbol{v}_r^{\mathcal{B}} \\
\boldsymbol{\omega}^{\mathcal{B}}
\end{bmatrix}
=
\begin{bmatrix}
u & v & w & p & q & r
\end{bmatrix}^{\mathsf T},
$$

where $\boldsymbol{v}_r^{\mathcal{B}}$ is the linear velocity through water and
$\boldsymbol{\omega}^{\mathcal{B}}$ is the body angular velocity. For a water current
$\boldsymbol{v}_c^{\mathcal{I}}$ expressed in NED, the rigid-body kinematics are

$$
\dot{\boldsymbol{p}}^{\mathcal{I}} = R\boldsymbol{v}_r^{\mathcal{B}} + \boldsymbol{v}_c^{\mathcal{I}},
\qquad
\dot R = R[\boldsymbol{\omega}^{\mathcal{B}}]_\times,
$$

where $[\boldsymbol{a}]_\times\boldsymbol{b}
= \boldsymbol{a}\times\boldsymbol{b}$. Thus, velocity over ground in body axes is

$$
\boldsymbol{v}_g^{\mathcal{B}} = \boldsymbol{v}_r^{\mathcal{B}} + R^{\mathsf T}\boldsymbol{v}_c^{\mathcal{I}}.
$$

The configured current is constant and horizontal. For speed $V_c$ and direction
$\beta$, measured clockwise from north toward the flow direction,

$$
\boldsymbol{v}_c^{\mathcal{I}} = V_c
\begin{bmatrix}
\cos\beta & \sin\beta & 0
\end{bmatrix}^{\mathsf T}.
$$

Current advects the vehicle through the inertial frame; it is added once in
kinematics. Dynamics and actuator inflow use velocity through water. This
simplified relative-motion model does not include current acceleration or shear
forces. A vehicle at rest relative to the water therefore drifts over ground
with the current.

### Dynamics

The simulator uses the following Fossen-style relative-motion equation:

$$
M\dot{\boldsymbol{\nu}}_r
+ C(M,\boldsymbol{\nu}_r)\boldsymbol{\nu}_r
+ D_l\boldsymbol{\nu}_r
+ D_q\bigl(|\boldsymbol{\nu}_r|\odot\boldsymbol{\nu}_r\bigr)
+ \boldsymbol{g}(\boldsymbol{p}^{\mathcal{I}},R)
= \boldsymbol{\tau},
\qquad M = M_{RB} + M_A.
$$

Here, $M_{RB}$ is the rigid-body mass matrix, $M_A$ is added mass,
$C$ represents Coriolis and centripetal effects, $D_l$ and $D_q$ describe linear
and quadratic damping, $\boldsymbol{g}$ is the restoring term, and
$\boldsymbol{\tau}$ is the total actuator wrench in body axes. Absolute values
and $\odot$ act componentwise. Damping coefficients represent positive resistance;
$D_q$ is diagonal and nonnegative.

For mass $m$, body-frame center of gravity $\boldsymbol{r}_G$, and inertia tensor
$I_G$ about the center of gravity,

$$
M_{RB} =
\begin{bmatrix}
mI_3 & -m[\boldsymbol{r}_G]_\times \\
m[\boldsymbol{r}_G]_\times & I_G-m[\boldsymbol{r}_G]_\times^2
\end{bmatrix}.
$$

Writing $M\boldsymbol{\nu}_r =
[\boldsymbol{h}_v^{\mathsf T},\boldsymbol{h}_\omega^{\mathsf T}]^{\mathsf T}$,
the implemented Coriolis matrix is

$$
C(M,\boldsymbol{\nu}_r) =
\begin{bmatrix}
0_{3\times3} & -[\boldsymbol{h}_v]_\times \\
-[\boldsymbol{h}_v]_\times & -[\boldsymbol{h}_\omega]_\times
\end{bmatrix}.
$$

Weight and buoyancy act at the centers of gravity and buoyancy, respectively.
For gravitational acceleration $g_0$, fluid density $\rho$, displaced volume
$\mathcal V$, and body-frame center of buoyancy $\boldsymbol{r}_B$,

$$
\boldsymbol{f}_W^{\mathcal{B}} = R^{\mathsf T}
\begin{bmatrix}0\\0\\mg_0\end{bmatrix},
\qquad
\boldsymbol{f}_B^{\mathcal{B}} = R^{\mathsf T}
\begin{bmatrix}0\\0\\-\rho g_0\mathcal V\end{bmatrix},
$$

$$
\boldsymbol{g} = -
\begin{bmatrix}
\boldsymbol{f}_W^{\mathcal{B}} + \boldsymbol{f}_B^{\mathcal{B}} \\
\boldsymbol{r}_G\times\boldsymbol{f}_W^{\mathcal{B}}
+ \boldsymbol{r}_B\times\boldsymbol{f}_B^{\mathcal{B}}
\end{bmatrix}.
$$

Displaced volume is constant by default. An optional simplified near-surface
model scales it with immersion depth; this is an approximation rather than a
general hull hydrostatics model.

Models with selected degrees of freedom use a selection matrix $S$, with
$\boldsymbol{\nu}_r=S\boldsymbol{\nu}_a$. The reduced dynamics are solved as

$$
(S^{\mathsf T}MS)\dot{\boldsymbol{\nu}}_a
= S^{\mathsf T}\left[
\boldsymbol{\tau}-C\boldsymbol{\nu}_r-D_l\boldsymbol{\nu}_r
-D_q(|\boldsymbol{\nu}_r|\odot\boldsymbol{\nu}_r)-\boldsymbol{g}
\right].
$$

Inactive body-velocity components remain zero. These constraints apply to
water-relative body motion and do not necessarily fix inertial-frame
coordinates. Underactuation alone does not imply a constrained degree of freedom.

Time stepping uses a first-order semi-implicit update: velocity is advanced
first, followed by position using the updated velocity and the starting attitude.
Attitude is advanced using
$R_{k+1}=R_k\exp([\boldsymbol{\omega}_{k+1}^{\mathcal{B}}]_\times\Delta t)$,
which preserves the rotation-matrix structure to numerical roundoff.

### Actuator models

Each actuator generates a force in its local frame. For mounting rotation $Q_i$
from actuator to body axes and body-frame position $\boldsymbol{r}_i$, its wrench is

$$
\boldsymbol{\tau}_i =
\begin{bmatrix}
Q_i\boldsymbol{f}_i \\
\boldsymbol{r}_i\times(Q_i\boldsymbol{f}_i)
\end{bmatrix},
\qquad
\boldsymbol{\tau}=\sum_i\boldsymbol{\tau}_i.
$$

#### Thrusters 
RPM commands are saturated and passed through a dead zone,
transport delay $T_d$, and optional first-order motor response. If $N_d$ is the
limited, delayed command and $\lambda>0$ is the motor pole,

$$
\dot N = \lambda(N_d-N),
\qquad N_d(t)=N_{\mathrm{limited}}(t-T_d).
$$

A zero pole selects ideal tracking of the delayed command. Thrust acts along the
thruster's local positive X axis, $\boldsymbol{f}_i=[T_i,0,0]^{\mathsf T}$.
Two thrust laws are available:

$$
T(N)=
\begin{cases}
a_+N^2+b_+|N|+c_+, & N>0,\\
0, & N=0,\\
a_-N^2+b_-|N|+c_-, & N<0,
\end{cases}
$$

or the propeller model, for nonzero $n=N/60$ in revolutions per second,

$$
T = s_T\rho\,n|n|D^4K_T
\left(1-\frac{u_r}{nP}\right).
$$

Here, $D$ is propeller diameter, $P$ is pitch, $K_T$ is the thrust coefficient,
and $s_T$ is a scale factor. The advance speed $u_r$ is the vehicle's through-water
surge velocity; mounting geometry determines the force axis and moment arm but
does not change this advance speed. Thrust is zero at zero RPM.

#### Rudder 
The rudder angle $\delta$ follows a saturated command with an optional
angular rate limit. For zero-angle mounting rotation $Q$ and offset
$\boldsymbol{r}$, the local through-water velocity is

$$
\boldsymbol{v}_{\mathrm{loc}} = Q^{\mathsf T}
\left(\boldsymbol{v}_r^{\mathcal{B}}+\boldsymbol{\omega}^{\mathcal{B}}\times\boldsymbol{r}\right).
$$

Let $U^2=v_{\mathrm{loc},x}^2+v_{\mathrm{loc},y}^2$ and
$\gamma=\operatorname{atan2}(v_{\mathrm{loc},y},v_{\mathrm{loc},x})$.
The angle of attack is $\alpha=\operatorname{wrap}_{[-\pi,\pi]}(\delta+\gamma)$.
The model uses effective lift and drag coefficients:

$$
L=k_L\alpha U^2,
\qquad D=(k_{D0}+k_{D2}\alpha^2)U^2,
$$

$$
\boldsymbol{f}_{\mathrm{rudder}} =
\begin{bmatrix}
-D\cos\gamma+L\sin\gamma \\
-D\sin\gamma-L\cos\gamma \\
0
\end{bmatrix}.
$$

The mounting transform maps this force into a body wrench, including its moment
about the body origin.

## Available sensors

Measurements are generated from the simulated vehicle state and share simulation
timestamps. Mounted sensors account for their fixed position and orientation
relative to the body, including rotational lever-arm effects where applicable.

| Sensor | Measurement |
| --- | --- |
| GNSS | Antenna latitude, longitude, and altitude, with optional UTM position and velocity over ground. |
| IMU | Sensor attitude, angular velocity, and accelerometer specific force, including gravity subtraction and acceleration due to the mounting offset. |
| DVL | Velocity over ground in bottom-track mode, or velocity through water in water-track mode. Output can use sensor axes, compensated body-origin velocity in body axes, or sensor-origin velocity in NED. |
| Odometer | Velocity through water at the sensor origin, expressed in sensor axes. |
| Depth cell | Sensor depth below the configured water surface. |
| Altimeter | A synthetic sinusoidal range between 2 and 4 m for testing. It does not model a seabed or depend on vehicle pose. |
| Thruster RPM | Actual motor RPM after command limiting, delay, and motor response. |
| Control-surface angle | Actual rudder angle after angle and rate limits, reported in degrees. |

Sensors are selected with `sensor_list`. GNSS, IMU, DVL, depth cell, and odometer
support configurable bias and Gaussian noise on their supported measurement
channels. The altimeter and actuator feedback have no added measurement noise.
The [example vehicle configuration](../config/vehicle.yaml) illustrates the
available model and sensor settings.

## Simulation clock

The package also provides a `sim_clock` node that publishes ROS simulation time on
`/clock`. It supplies a common time reference for simulated vehicles and other
nodes using ROS time, with a configurable clock frequency and speedup factor.
One clock publisher serves each simulation instance within its ROS domain.
