# Rotary_Inverted_Pendulum
Dynamics and Control of Rotary Inverted Pendulum 

The dynamics of the Rotary Inverted Pendulum system are first formulated and then linearlised assuming a regulation problem. Refer to `OpenLoopDyn.m` for the open loop (non-linear) dynamic response of the system. The open loop response is shown below:

![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/Open%20Loop%20Dynamics.png)

For now, the repository contains code for only the regulation problem in the system and its disturbance rejection. There are 2 more branches in this repository: `InvDynVer` which formulates the dynamics of the system using Euler Lagrange equations of the 2nd Kind and `ParVar` which explores the parameter variations of the system. Work still needs to be done for the paramater variation part of the system after which both the branches will be merged to the main branch.

**CONTROLLER USED**: PD Control

**CONTROL OBJECTIVE**: Regulation and disturbance rejection

**Actuation**: Underactuated (Rotary Arm Actuated, Pendulum joint passive)

<h2>MODEL ARCHITECTURE</h2>

![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/Control%20Architecture.jpg)

<h2>SOME RESULTS</h2>

* No external disturbance
![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/NoDisturbance_Response.png)

* Impulse Disturbance on Pendulum
![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/RA%20Inv%20Pend%20Joint%20Disturbed/ImpulseDisturbance_Response.png)

* Harmonic Loading on Pendulum
![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/RA%20Inv%20Pend%20Joint%20Disturbed/HarmonicDisturbance_Response.png)

* Harmonic Loading on Rotary Arm
![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/RA%20Base%20Joint%20Disturbed/HarmonicDisturbance_Response.png)

* Static Loading on Rotary Arm
![sys](https://github.com/average-engineer/Rotary_Inverted_Pendulum/blob/main/Results/RA%20Base%20Joint%20Disturbed/StaticDisturbance_Response.png)



*Main executable file:* `RotInvPend.m`

*State Space Model of the linearelised model:* `ClosedLoopDyn1.m`
