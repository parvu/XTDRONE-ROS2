# 46. Dynamics model

!!! success "Rewritten for Gazebo Harmonic"
    Background chapter. The models are the same as in Classic (Gazebo Harmonic's systems were ported from the
    RotorS / PX4 `sitl_gazebo` plugins), but the plugin names and the physics engine differ.

The quality of the dynamics model largely decides how much a simulator is worth for UAVs. Gazebo Harmonic
integrates the rigid-body equations with **DART** (through gz-physics) instead of ODE. The step is 4 ms
(`max_step_size` 0.004, 250 Hz), which PX4 needs for lockstep.

## Motor–rotor model (`gz-sim-multicopter-motor-model-system`)

All rotors and propellers (multirotors, the pusher of fixed-wings and VTOLs) use `MulticopterMotorModel`. The
rotor force \(\boldsymbol F\) splits into thrust \(\boldsymbol T\) and the in-plane drag (H-force) \(\boldsymbol H\).
The moment splits into the rolling moment \(\boldsymbol M_R\) and the drag torque \(\boldsymbol M_D\):

\[
\boldsymbol T=-\omega^{2}C_{T}\,\boldsymbol e_{z,b},\qquad
\boldsymbol H=-\omega C_{D}\,\boldsymbol v^{\perp},\qquad
\boldsymbol M_{R}=-\zeta\,\omega C_{R}\,\boldsymbol v^{\perp},\qquad
\boldsymbol M_{D}=-\zeta C_{M}\,\boldsymbol T
\]

\(\omega\) is the rotor speed, \(\boldsymbol v^{\perp}\) the projection of the airspeed onto the rotor plane, and
\(\zeta=+1\) for counter-clockwise, \(-1\) for clockwise rotors. The SDF parameters:

| Symbol | SDF element | Physical meaning |
|---|---|---|
| \(C_T\) | `<motorConstant>` | \(C_{t0}\rho d^4/(2\pi)^2\) |
| \(C_D\) | `<rotorDragCoefficient>` | \(\tfrac{1}{16}\rho m c C_{d0} d^2\) |
| \(C_R\) | `<rollingMomentCoefficient>` | \((\theta_0/48+\theta_1/64)\rho n k c d^3\) |
| \(C_M\) | `<momentConstant>` | \(C_{m0} d / C_{t0}\) |
| | `<timeConstantUp>`, `<timeConstantDown>` | first-order lag of the motor speed |
| | `<maxRotVelocity>` | speed limit (rad/s) |
| | `<rotorVelocitySlowdownSim>` | visual slow-down factor only |

\(\rho\) is the air density, \(d\) the rotor diameter, \(m\) the number of blades, \(c\) the mean chord, \(k\) the
lift slope, \(C_{t0}, C_{d0}, C_{m0}\) the static thrust, drag and moment coefficients, and \(\theta_0, \theta_1\) the
blade root pitch and twist. For a hovering vehicle of mass \(m_v\) with \(n\) rotors,
\(\omega_\text{hover}=\sqrt{m_v g/(n C_T)}\). The XTDrone typhoon (≈2.3 kg, \(C_T\)=8.55e-6) hovers at about 660 rad/s.

## Aerodynamic surfaces (`gz-sim-lift-drag-system`)

Wings, elevator and rudder of the plane and VTOLs use `LiftDrag`. Lift, drag and pitching moment are linear in
the angle of attack \(\alpha\) up to the stall angle and use different slopes after it:

\[
L=\tfrac12 C_L\rho V^2 S,\; C_L=C_{L\alpha}(\alpha-\alpha_0)+C_{L\delta}\,\delta,\qquad
D=\tfrac12 C_D\rho V^2 S,\; C_D=C_{D\alpha}(\alpha-\alpha_0),\qquad
M=\tfrac12 C_m\rho V^2 S,\; C_m=C_{m\alpha}(\alpha-\alpha_0)+C_{m\delta}\,\delta
\]

Parameters: `<link_name>`, `<air_density>`, `<area>`, `<a0>`, `<cla>`, `<cda>`, `<cma>`, `<alpha_stall>`,
`<cla_stall>`, `<cda_stall>`, `<cma_stall>`, `<cp>`, `<forward>`, `<upward>`, `<control_joint_name>`,
`<control_joint_rad_to_cl>`, `<cm_delta>`. Note that Gazebo's `a0` is the *zero-lift* angle of attack with the
sign convention shown above.

## Vehicles

- **Multirotor**: a 6-DOF rigid body with \(\sum(\boldsymbol F_i+\boldsymbol D+\boldsymbol G)=m\boldsymbol a\) and
  \(\sum(\boldsymbol M_i+\boldsymbol r_i\times\boldsymbol F_i)=\boldsymbol J\dot{\boldsymbol\Omega}+\boldsymbol\Omega\times\boldsymbol J\boldsymbol\Omega\).
  Body drag \(\boldsymbol D\) is neglected unless wind effects are enabled (chapter 41).
- **Fixed-wing** (`rc_cessna`): LiftDrag on wings and tail, one `MulticopterMotorModel` as propeller, control
  surfaces driven by `JointPositionController`.
- **VTOL**: `standard_vtol` (quadplane: four lift rotors + pusher), `quadtailsitter` and `tiltrotor` (tilting rotor
  joints with position controllers), each with LiftDrag surfaces.
