# From a constrained Lagrangian to the kite simulator

This document derives the equations implemented in `kite_sim/physics/`. It
starts with a rigid body, adds two taut lines through Lagrange multipliers,
solves for their tensions, and then expresses attitude using a unit quaternion.
The aerodynamic closure uses Fechner's three-surface force model.

The simulated kite is an illustrative rigid body. The coefficients, mass
layout and effective bridles are not measurements of a particular production
kite. The mathematics of the constraints does not establish aerodynamic accuracy.

## 1. Assumptions and notation

- The kite is rigid, with constant mass $m$ and a constant body-frame inertia
  tensor $J$ about its centre of mass (COM).
- Two massless, inextensible lines connect fixed ground anchors to fixed body
  attachment points. Their prescribed lengths can change through the motor.
- Both lines must remain taut. The game stops if the solution demands a line
  that pushes rather than pulls. Slack motion and re-tension impacts are absent.
- Gravity is uniform. Aerodynamics are quasi-steady distributed forces; there
  is no flexible canopy, dynamic stall, line drag, KCU or winch model.

| Symbol | Meaning | Frame / units |
| --- | --- | --- |
| $\mathbf r,\mathbf v$ | COM position and velocity | Ground, m and m/s |
| $R$ | Rotation from body to ground | $SO(3)$ |
| $\boldsymbol\omega$ | Angular velocity | Body, rad/s |
| $J$ | Symmetric positive-definite COM inertia | Body, kg m² |
| $\mathbf b_i$ | Effective line attachment relative to COM | Body, m |
| $\mathbf A_i$ | Fixed ground anchor | Ground, m |
| $L_i(t)$ | Prescribed line length | m |
| $\mathbf s_j$ | Aerodynamic force location relative to COM | Body, m |
| $\mathbf F_a,\boldsymbol\tau_a$ | Net aerodynamic force and COM torque | Ground and body respectively |

The ground frame has X forward, Y left and Z up. Body x points toward the nose,
y from the D side to the C side, and z from canopy top toward the side-plane
centre. Both frames are right-handed. All body lever arms use the COM as origin.
The renderer alone converts ground coordinates to x-forward, y-right, z-down.

Define the cross-product matrix by $[\mathbf x]_\times\mathbf y=\mathbf x\times\mathbf y$.
Since angular velocity is expressed in the body frame,

$$
\dot R=R[\boldsymbol\omega]_\times.
$$

## 2. Kinetic energy, potential and the augmented Lagrangian

For a material point with fixed body coordinate $\mathbf s$,

$$
\mathbf x=\mathbf r+R\mathbf s,
\qquad
\dot{\mathbf x}=\mathbf v+R(\boldsymbol\omega\times\mathbf s).
$$

Integrating $\tfrac12|\dot{\mathbf x}|^2$ over mass gives

$$
T=\frac12m\mathbf v^T\mathbf v+
\frac12\boldsymbol\omega^TJ\boldsymbol\omega,
\qquad
J=\int\left[(\mathbf s^T\mathbf s)I-\mathbf s\mathbf s^T\right]\,dm.
$$

The translational/rotational cross term vanishes because the origin is COM:
$\int\mathbf s\,dm=0$. Uniform gravity therefore has potential

$$
V=mg\,\mathbf e_z^T\mathbf r=mgz,
\qquad \mathcal L=T-V.
$$

For line $i$, define

$$
\mathbf a_i=\mathbf r+R\mathbf b_i-\mathbf A_i,
\qquad
g_i=\frac12(\mathbf a_i^T\mathbf a_i-L_i^2)=0.
$$

The augmented Lagrangian is

$$
\boxed{\mathcal L_{\rm aug}=\mathcal L-\sum_{i=0}^1\lambda_i g_i.}
$$

With this sign convention, positive $\lambda_i$ produces a force
$-\lambda_i\mathbf a_i$, directed from attachment toward anchor. Since
$|\mathbf a_i|=L_i$, the physical tension is

$$
\boxed{T_i=\lambda_iL_i.}
$$

The multiplier has units N/m, not N. Two independent scalar constraints remove
two of the free rigid body's six mechanical degrees of freedom, leaving four.
A fixed COM radius cannot also be imposed without changing the model.

## 3. Translational Euler–Lagrange equation

Aerodynamic force is a nonconservative generalized force. Applying
Euler–Lagrange to $\mathbf r$ gives

$$
\frac{d}{dt}\frac{\partial\mathcal L}{\partial\dot{\mathbf r}}
-\frac{\partial\mathcal L}{\partial\mathbf r}
=\mathbf F_a-\sum_i\lambda_i\frac{\partial g_i}{\partial\mathbf r}.
$$

Here $\partial g_i/\partial\mathbf r=\mathbf a_i$, so

$$
m\dot{\mathbf v}+mg\mathbf e_z=\mathbf F_a-\sum_i\lambda_i\mathbf a_i.
$$

Defining $\mathbf F=\mathbf F_a-mg\mathbf e_z$ simply moves the derivative of
the potential to the right-hand side:

$$
\boxed{\dot{\mathbf v}=\frac1m\left(\mathbf F-\sum_i\lambda_i\mathbf a_i\right).}
$$

**Gravity is counted once.** Adding another gravity generalized force while
retaining its contribution on the left would double count it. Uniform gravity
produces no torque about the COM.

## 4. Rotational equation from the same variational principle

An infinitesimal body rotation $\boldsymbol\eta$ has

$$
\delta R=R[\boldsymbol\eta]_\times,
\qquad
\delta\boldsymbol\omega=\dot{\boldsymbol\eta}
+\boldsymbol\omega\times\boldsymbol\eta.
$$

The second identity is why three independent copies of a scalar rotational
Euler–Lagrange equation are insufficient: the body coordinate axes rotate.
Taking variations of rotational kinetic energy, integrating by parts and making
$\boldsymbol\eta$ vanish at the endpoints gives

$$
\delta\int T_{\rm rot}\,dt
=-\int\left[J\dot{\boldsymbol\omega}
+\boldsymbol\omega\times(J\boldsymbol\omega)\right]^T
\boldsymbol\eta\,dt.
$$

For the constraint,

$$
\delta g_i=\mathbf a_i^TR(\boldsymbol\eta\times\mathbf b_i)
=\mathbf h_i^T\boldsymbol\eta,
\qquad
\mathbf h_i=\mathbf b_i\times(R^T\mathbf a_i).
$$

Aerodynamic virtual work is $\boldsymbol\tau_a^T\boldsymbol\eta$. Stationarity
of the augmented action with this virtual work therefore gives

$$
\boxed{J\dot{\boldsymbol\omega}+\boldsymbol\omega\times(J\boldsymbol\omega)
=\boldsymbol\tau_a-\sum_i\lambda_i\mathbf h_i.}
$$

The constraint torque is exactly the attachment force moment:
$\mathbf b_i\times R^T(-\lambda_i\mathbf a_i)=-\lambda_i\mathbf h_i$.
This is the rigid-body Euler equation, obtained here through Lagrange's method.
Define

$$
\mathbf k=\boldsymbol\tau_a-\boldsymbol\omega\times(J\boldsymbol\omega),
\qquad
\boxed{\dot{\boldsymbol\omega}=J^{-1}
\left(\mathbf k-\sum_i\lambda_i\mathbf h_i\right).}
$$

The full tensor matters: off-diagonal terms are generally nonzero unless body
axes coincide with principal inertia axes. No diagonal-inertia assumption is made.

### Connection to Euler-angle coordinates

For a local ZYX parametrization $R=R_z(\psi)R_y(\beta)R_x(\alpha)$, let
$\boldsymbol\xi=(\alpha,\beta,\psi)^T$. Then

$$
\boldsymbol\omega=E(\alpha,\beta)\dot{\boldsymbol\xi},\qquad
E=\begin{bmatrix}
1&0&-\sin\beta\\
0&\cos\alpha&\sin\alpha\cos\beta\\
0&-\sin\alpha&\cos\alpha\cos\beta
\end{bmatrix}.
$$

The Euler-angle Lagrangian has rotational energy
$\tfrac12\dot{\boldsymbol\xi}^TE^TJE\dot{\boldsymbol\xi}$, generalized torque
$E^T\boldsymbol\tau_a$ and constraint gradient $E^T\mathbf h_i$.
Its three Euler–Lagrange equations can be written

$$
E^T\left[J\dot{\boldsymbol\omega}+\boldsymbol\omega\times(J\boldsymbol\omega)
-\boldsymbol\tau_a+\sum_i\lambda_i\mathbf h_i\right]=0.
$$

Where $E$ is invertible these are the same rotational equations. At
$\cos\beta=0$, Euler angles become singular. This motivates quaternions;
it does not indicate a physical singularity of the kite.

## 5. Differentiate the constraints and solve the multipliers

Let the attachment velocity be

$$
\mathbf u_i=\dot{\mathbf a}_i
=\mathbf v+R(\boldsymbol\omega\times\mathbf b_i).
$$

The velocity constraint is

$$
\dot g_i=\mathbf a_i^T\mathbf u_i-L_i\dot L_i
=\mathbf a_i^T\mathbf v+\mathbf h_i^T\boldsymbol\omega-L_i\dot L_i=0.
$$

Differentiate again. Because $\mathbf b_i$ is fixed in the rotating body,

$$
\ddot{\mathbf a}_i=\dot{\mathbf v}
+R\left[\dot{\boldsymbol\omega}\times\mathbf b_i
+\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf b_i)\right],
$$

and

$$
0=\ddot g_i=\mathbf u_i^T\mathbf u_i+\mathbf a_i^T\ddot{\mathbf a}_i
-\dot L_i^2-L_i\ddot L_i.
$$

Substituting the force and rotational equations, and using
$\mathbf a_i^TR(\dot{\boldsymbol\omega}\times\mathbf b_i)
=\mathbf h_i^T\dot{\boldsymbol\omega}$, gives

$$
\boxed{M\boldsymbol\lambda=\mathbf c,}
$$

with

$$
\boxed{M_{ij}=\frac{\mathbf a_i^T\mathbf a_j}{m}
+\mathbf h_i^TJ^{-1}\mathbf h_j,}
$$

$$
\boxed{c_i=\mathbf u_i^T\mathbf u_i+\frac{\mathbf a_i^T\mathbf F}{m}
+\mathbf h_i^TJ^{-1}\mathbf k
+\mathbf a_i^TR[\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf b_i)]
-\dot L_i^2-L_i\ddot L_i.}
$$

This is the section of the model that receives the aerodynamic force and torque.
For two lines the solve is only 2×2. Explicitly, for
$D=M_{00}M_{11}-M_{01}^2$,

$$
\lambda_0=\frac{M_{11}c_0-M_{01}c_1}{D},\qquad
\lambda_1=\frac{M_{00}c_1-M_{01}c_0}{D}.
$$

The code uses a linear solve rather than forming an inverse. With positive
mass, positive-definite $J$ and independent constraint gradients, $M$ is
positive definite: it is the constraint Jacobian times inverse generalized
mass times the Jacobian transpose. Dependent or nearly dependent geometry is
rejected, not regularized with an arbitrary force.

### Tension-only validity and energy check

An ideal line cannot carry compression. A complete slack/taut model would obey
$g_i\le0$, $\lambda_i\ge0$, $\lambda_i g_i=0$, with suitable switching and impact
handling. This implementation solves only the two-taut-line branch and stops
when either multiplier is negative.

The mechanical power delivered by the constraints is

$$
P_{\rm lines}=-\sum_i\lambda_i
(\mathbf a_i^T\mathbf v+\mathbf h_i^T\boldsymbol\omega)
=-\sum_i\lambda_iL_i\dot L_i=-\sum_iT_i\dot L_i.
$$

Fixed ideal lines do no work. Prescribed moving lines can exchange energy with
the kite, which is why the length-rate terms must not be dropped. Consequently,

$$
\frac{d}{dt}(T+V)=\mathbf F_a^T\mathbf v+
\boldsymbol\tau_a^T\boldsymbol\omega-\sum_iT_i\dot L_i.
$$

## 6. Transfer attitude to a unit quaternion

Use the Hamilton, scalar-first quaternion
$q=(q_0,\mathbf q_v)=(w,x,y,z)$ with $q^Tq=1$. It represents the **active
body-to-ground** rotation

$$
\boxed{R(q)=(q_0^2-\mathbf q_v^T\mathbf q_v)I
+2\mathbf q_v\mathbf q_v^T+2q_0[\mathbf q_v]_\times.}
$$

For quaternions $p=(p_0,\mathbf p)$ and $q=(q_0,\mathbf q)$,

$$
p\otimes q=(p_0q_0-\mathbf p^T\mathbf q,
\ p_0\mathbf q+q_0\mathbf p+\mathbf p\times\mathbf q).
$$

If starting from ZYX Euler angles, conversion is

$$
q=q_z(\psi)\otimes q_y(\beta)\otimes q_x(\alpha),
\quad
q_x(\alpha)=(\cos(\alpha/2),\sin(\alpha/2),0,0),
$$

with analogous y and z rotations. The order matches $R_zR_yR_x$.

A small **body-frame** rotation multiplies on the right. Dividing its
increment by time yields

$$
\boxed{\dot q=\frac12q\otimes(0,\boldsymbol\omega)
=\frac12Q(q)\boldsymbol\omega,}
\qquad
Q(q)=\begin{bmatrix}-\mathbf q_v^T\\q_0I+[\mathbf q_v]_\times\end{bmatrix}.
$$

Equivalently,

$$
\dot q_0=-\tfrac12\mathbf q_v^T\boldsymbol\omega,\qquad
\dot{\mathbf q}_v=\tfrac12(q_0\boldsymbol\omega+\mathbf q_v\times\boldsymbol\omega).
$$

Since $q^TQ(q)=0$, $d(q^Tq)/dt=0$ analytically. For unit q,
$Q^TQ=I$ and $\boldsymbol\omega=2Q^T\dot q$. One can therefore write the same
Lagrangian directly in quaternion coordinates:

$$
\mathcal L_q=\frac12m|\dot{\mathbf r}|^2
+2\dot q^TQJQ^T\dot q-mgz,
$$

$$
\mathcal L_{q,\rm aug}=\mathcal L_q-\sum_i\lambda_i g_i(\mathbf r,q,t)
-\frac\mu2(q^Tq-1).
$$

The additional multiplier $\mu$ enforces the redundant coordinate's unit norm;
it is not a physical line tension. Projecting these Euler–Lagrange equations
onto the three-dimensional rotational tangent space recovers the body-angular-
velocity equation derived above. The code avoids solving for $\mu$ by integrating
$\boldsymbol\omega$ and the quaternion kinematics together.

**What changes:** three Euler angles become four numbers with one constraint;
there is no Euler-angle gimbal lock, and $q$ and $-q$ describe the same rotation.
**What does not change:** the inertia, force, torque, line constraints and
multiplier equations. Every occurrence of $R$ simply uses $R(q)$. Quaternion
normalization does not enforce either line constraint.

## 7. Aerodynamic forces and torques: Fechner-style closure

The mechanics above do not determine aerodynamic loads. This game uses a rigid
adaptation of the top/side force construction in Fechner et al. (2015), Eqs. 47–59.
The paper's complete model uses moving mass points and spring/damper connections;
this game keeps their load geometry rigid and uses two ideal ground lines.

### Geometry and inertia estimate

Before shifting to COM, define $w_r=\text{width}\times\text{relative_width}$ and

$$
A=(n w_r,0,0),\quad B=(0,0,-h),\quad
C=(0,w_r/2,0),\quad D=(0,-w_r/2,0),
$$

where $n$ is `nose_distance`, $h$ is `height`. The default mass fractions are
$\gamma=0.47$ at A, $0.4(1-\gamma)$ at B and $0.3(1-\gamma)$ at each side.
Compute $\mathbf r_{\rm COM}=\sum_jm_j\mathbf x_j/m$ and subtract it from all
four points. With no supplied tensor, the inertia estimate is

$$
J=\sum_jm_j[(\mathbf s_j^T\mathbf s_j)I-\mathbf s_j\mathbf s_j^T].
$$

A configured 3×3 `inertia` overrides this estimate. Physical principal moments
must also satisfy the triangle inequalities. There are aerodynamic forces at
B, C and D; A contributes to the mass layout but carries no aerodynamic load.

### Local apparent wind

At each aerodynamic site $j$,

$$
\boxed{\mathbf w_j=R^T[\mathbf W(\mathbf r+R\mathbf s_j)-\mathbf v]
-\boldsymbol\omega\times\mathbf s_j.}
$$

The final term makes rotation change local airflow, and therefore the resulting
moment. Using only COM wind would omit this effect. Sample wind at each site's
height. Project it into the relevant surface plane:

$$
\mathbf w_{B,xz}=\mathbf w_B-(\mathbf w_B\cdot\mathbf e_y)\mathbf e_y,
\quad
\mathbf w_{C,xy}=\mathbf w_C-(\mathbf w_C\cdot\mathbf e_z)\mathbf e_z,
\quad
\mathbf w_{D,xy}=\mathbf w_D-(\mathbf w_D\cdot\mathbf e_z)\mathbf e_z.
$$

Define the paper's unsigned geometric incidence, in radians, by

$$
\vartheta(\mathbf u)=\pi-\arccos\left(\frac{\mathbf u\cdot\mathbf e_x}{|\mathbf u|}\right).
$$

The implemented incidences are

$$
\alpha_B=\vartheta(\mathbf w_{B,xz})-\alpha_d+\alpha_0,\quad
\alpha_C=\vartheta(\mathbf w_{C,xy})-\alpha_s+\alpha_{s0},\quad
\alpha_D=\vartheta(\mathbf w_{D,xy})+\alpha_s+\alpha_{s0}.
$$

Defaults are $\alpha_0=4°$, $\alpha_{s0}=10°$ and
$\alpha_d=10°\,d$ for normalized depower $d\in[0,1]$.
The game uses a 10° incidence range instead of the source KCU's 31° range.
With the illustrative rigid geometry, the larger range can unload a line while
steering or changing depower in gusts. The smaller range retains steering load;
it is a gameplay default, not a new measured aerodynamic coefficient. Thus d=1
is maximum **game** depower, not a claim of complete depower of a physical kite.
The source KCU depower offset is not used: here d=0 means fully powered.
A quasi-steady differential side-incidence surrogate is

$$
\alpha_s=-\text{steering\_twist}\,
\frac{\delta_m/\delta_{m,\max}}{1+k_{ds}d}.
$$

The default maximum is **16.83°**, from the reference settings. The minus sign
maps this game's positive-left motor travel to the source's side-incidence
convention. The opposite sign would make aerodynamic steering fight the line
geometry and reverse the intended turn.

This adds an effective aerodynamic response to differential steering while the
mass layout, inertia and attachment positions remain rigid. It is not a flexible
structural simulation. Both line geometry and side-incidence loading contribute
to steering; their combined gain is an illustrative game setting, not calibrated
hardware. Setting `steering_twist_deg: 0` disables the incidence response.
There is no fitted turn-rate law or direct artificial yaw torque.

### Polar coefficients and force vectors

CL and CD are linearly interpolated from the tables in `configs/default.yaml`.
These are a frozen numeric snapshot of OpenSourceAWE/KiteUtils.jl settings,
accessed 2026-11-08; the simulator does not fetch them at runtime. Linear
interpolation is an explicit adaptation of the reference interpolation.

| Lift angle (degrees) | -180 | -160 | -90 | -20 | -10 | -5 | 0 | 20 | 40 | 90 | 160 | 180 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CL | 0 | .5 | 0 | .08 | .125 | .15 | .2 | 1 | 1 | 0 | -.5 | 0 |

| Drag angle (degrees) | -180 | -170 | -140 | -90 | -20 | 0 | 20 | 90 | 140 | 170 | 180 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CD | .5 | .5 | .5 | 1 | .2 | .1 | .2 | 1 | .5 | .5 | .5 |

Angles wrap to [-180°,180°). The table's post-peak lift reduction is a static
approximation, not a dynamic stall model. The unsigned-incidence convention
also limits interpretation of inverted or reversed-flow flight.

Let $A$ now denote projected aerodynamic area, $s=A_{side}/A=0.306$ and
$K_D=0.93(1-s)$, following the published drag correction. Body-frame lift is

$$
\mathbf f_{L,B}=\tfrac12\rho A|\mathbf w_{B,xz}|^2C_L(\alpha_B)
\frac{\mathbf w_B\times\mathbf e_y}{|\mathbf w_B\times\mathbf e_y|},
$$

$$
\mathbf f_{L,C}=\tfrac12\rho sA|\mathbf w_{C,xy}|^2C_L(\alpha_C)
\frac{\mathbf w_C\times\mathbf e_z}{|\mathbf w_C\times\mathbf e_z|},
$$

$$
\mathbf f_{L,D}=\tfrac12\rho sA|\mathbf w_{D,xy}|^2C_L(\alpha_D)
\frac{\mathbf e_z\times\mathbf w_D}{|\mathbf e_z\times\mathbf w_D|}.
$$

For each site, with $A_B=A$ and $A_C=A_D=sA$,

$$
\mathbf f_{D,j}=\tfrac12\rho K_D A_j|\mathbf w_j|^2C_D(\alpha_j)
\frac{\mathbf w_j}{|\mathbf w_j|},\qquad
\mathbf f_j=\mathbf f_{L,j}+\mathbf f_{D,j}.
$$

Lift uses projected speed, drag uses full speed. Zero projected wind gives zero
lift; zero wind gives zero drag. Direction normalization is numerically guarded.
Finally,

$$
\boxed{\mathbf F_a=R\sum_{j=B,C,D}\mathbf f_j,\qquad
\boldsymbol\tau_a=\sum_{j=B,C,D}\mathbf s_j\times\mathbf f_j.}
$$

These are the two inputs to Sections 3–5. No separate intrinsic pitching-moment
coefficient is included; all aerodynamic torque comes from these moment arms.

## 8. Motor, prescribed lengths and wind

Motor travel $\delta_m$ is in metres, with a delayed command and second-order servo:

$$
\ddot\delta_m=\omega_m^2(\delta_{m,ref,delayed}-\delta_m)
-2\zeta_m\omega_m\dot\delta_m.
$$

The game defaults to $\omega_m=10$ rad/s and $\zeta_m=1$ (critical damping),
with a 0.1 s transport delay. A keyboard press requests 0.12 m of motor travel.
The former 78 rad/s servo and 0.03 m keyboard request produced an initial
acceleration of $78^2\times0.03=182.52$ m/s². For the ideal inextensible lines,
that could demand negative tension as soon as a depowered steering command
arrived. The new start-from-rest acceleration is $10^2\times0.12=12$ m/s²;
reversals are also exercised in the game-control tests. These servo settings are
chosen for the illustrative game, not claimed as measured hardware coefficients.

The code limits speed and stroke and rate-limits depower. With gain $k_l$,

$$
L_0=l-\tfrac12k_l\delta_m,\qquad L_1=l+\tfrac12k_l\delta_m,
$$

$$
\dot L_{0,1}=\mp\tfrac12k_l\dot\delta_m,\qquad
\ddot L_{0,1}=\mp\tfrac12k_l\ddot\delta_m.
$$

These actual servo derivatives enter the multiplier solve, rather than the raw
keyboard target. Positive motor travel produces a left turn for the default
geometry. Abrupt commands can demand negative tension and stop the game.
Hard saturation is an idealization: projection can make velocity corrections
at those nonsmooth events. There is no elastic actuator/line impact model.

The ambient wind has fixed heading $\chi$ and height shear:

$$
\mathbf W(z)=U(z)(\cos\chi,\sin\chi,0),\qquad
U(z)=\max\left(0,U_{ref}\left[\frac{\max(z,z_{min})}{z_{ref}}\right]^p(1+\zeta)\right).
$$

Optional scalar gusts use the exact discrete Ornstein–Uhlenbeck update

$$
\zeta_{n+1}=a\zeta_n+\sigma\sqrt{1-a^2}\,\epsilon_n,
\quad a=e^{-\Delta t/\tau},\quad \epsilon_n\sim\mathcal N(0,1).
$$

The random-generator state belongs to the simulator state, making seeded reset
repeatable. This is spatial shear plus correlated speed gusts, not a complete
three-dimensional turbulent wind field.

## 9. Complete integration algorithm

The integrated state is $(\mathbf r,\mathbf v,q,\boldsymbol\omega)$ plus motor,
depower and delay state. Each 5 ms default step:

1. Delay the requested motor command and advance the explicit wind RNG once.
2. At each RK4 stage, evaluate servo derivatives, local wind, surface forces,
   aerodynamic torque, gravity, line lengths/rates/accelerations and $M\lambda=c$.
3. Reject a stage requiring negative line tension. Otherwise evaluate
   $\dot{\mathbf r}=\mathbf v$, $\dot{\mathbf v}$, $\dot{\boldsymbol\omega}$ and
   $\dot q=\tfrac12q\otimes(0,\boldsymbol\omega)$ and complete the RK4 update.
4. Apply actuator limits, normalize q, and correct accumulated line-constraint
   position and velocity drift as below.
5. Check ground contact at the model's geometry/attachment points. Return the
   new state or an explicit model-envelope stop. A stopped state stays stopped
   until reset.

### Position projection

Solve $M\boldsymbol\nu=\mathbf g$ and apply a mass-weighted correction

$$
\delta\mathbf r=-\frac1m\sum_i\nu_i\mathbf a_i,\qquad
\delta\boldsymbol\eta=-J^{-1}\sum_i\nu_i\mathbf h_i.
$$

Update q using a body rotation increment:

$$
q\leftarrow q\otimes\left(\cos\frac{|\delta\boldsymbol\eta|}{2},
\frac{\sin(|\delta\boldsymbol\eta|/2)}{|\delta\boldsymbol\eta|}\delta\boldsymbol\eta\right).
$$

Iterate because the constraints are nonlinear. The default criterion is
$\max_i|g_i|/L_i<10^{-10}$ m, with a finite iteration limit and explicit failure.

### Velocity projection

For residual $e_i=\mathbf a_i^T\mathbf v+\mathbf h_i^T\boldsymbol\omega-L_i\dot L_i$,
solve $M\boldsymbol\nu=\mathbf e$ and correct

$$
\mathbf v\leftarrow\mathbf v-\frac1m\sum_i\nu_i\mathbf a_i,\qquad
\boldsymbol\omega\leftarrow\boldsymbol\omega-J^{-1}\sum_i\nu_i\mathbf h_i.
$$

These are numerical drift corrections, not extra springs, and do not replace
the acceleration-level multiplier solve. RK4 plus projection is not claimed to
be an exactly energy-conserving variational integrator. Timestep convergence is
tested, especially because hard motor limits are nonsmooth.

Initialization first positions the airborne body consistently with both line
lengths while preserving the requested initial attitude, then projects velocity.
The COM need not be exactly one line length from the ground anchor.

## 10. Display, tests and limits

The mesh uses the simulated quaternion for nose, span and canopy normal.
Its shape and short bridle branches are illustrative, not a flexible structural
solution. Apparent size uses a fixed angular-scale correction; distance and
physical foreshortening still matter. No force-dependent resizing is applied.

The instrument view reports COM position and the two individual tensions.
The main pull display is

$$
F_{pull}=\left|\sum_i\lambda_i\mathbf a_i\right|,
$$

which is the resultant ground force, not necessarily the sum of tension magnitudes.
The 1,000 N instrument gauge scale is a display choice, not a safety limit.
Derived elevation/azimuth/course can be ambiguous directly at zenith; the
Cartesian/quaternion dynamics themselves have no corresponding singularity.

Tests exercise force/torque balance with a full inertia tensor, differentiated
moving-line constraints, constraint power, quaternion conventions, mirror
steering, delay, numerical convergence, deterministic wind and camera geometry.
They do not establish that the illustrative kite accurately predicts real flight.

For a particular physical kite one would need measured mass/COM/inertia,
attachment geometry, motor/line mapping, aerodynamic polars/force centres and
synchronized control, wind and motion data. Flexible deformation, changing
centres of pressure, dynamic stall, line mass/sag/drag, slack recovery and contact
remain outside this model.

## Sources and code map

- Fechner, van der Vlugt, Schreuder & Schmehl (2015), *Dynamic Model of a Pumping
  Kite Power System*, [arXiv:1406.6218v3](https://arxiv.org/abs/1406.6218v3).
  Geometry/mass layout and three-surface load construction; this game is a rigid
  adaptation, not the complete spring/damper model or its validation results.
- [OpenSourceAWE/KiteUtils.jl reference settings](https://github.com/OpenSourceAWE/KiteUtils.jl/blob/main/data/settings.yaml):
  polar knots and reference parameters, numeric snapshot accessed 2026-11-08.
- Fagiano et al., *Automatic Crosswind Flight of Tethered Wings for Airborne Wind
  Energy: Modeling, Control Design, and Experimental Results*,
  [arXiv:1301.1064](https://arxiv.org/abs/1301.1064): second-order steering servo,
  Eq. 20. Delay, limits and prescribed two-line mapping are additional assumptions.

| Code | Responsibility |
| --- | --- |
| `physics/rigid_params.py` | Configuration, rigid geometry and inertia estimate |
| `physics/quaternion.py` | Hamilton product, rotation, body rotation increments |
| `physics/constraints.py` | Multiplier solve and position/velocity projection |
| `physics/fechner_aero.py` | Polar interpolation and distributed aerodynamic loads |
| `physics/kite.py` | State, gravity, assembly, initialization and RK4 |
| `physics/actuators.py` | Motor servo, transport delay and limits |
| `physics/wind.py` | Height shear and seeded OU gusts |
| `app/` | Keyboard input, scene and instrument rendering |

Paths in the table are relative to `kite_sim/`.
