# INVERTED PENDULUM CONTROL USING GENETIC ALGORITHM
## Welcome!

This is a project implementing a genetic algorithm to determine parameters of an inverted pendulum controller.
In other words - **the point is to keep inverted pendulum in a vertical upright position without knowing maths and physics behind it.**

The **main.py** file is the main file which you can execute in order to start a simulation and which contains basic system configuration
where you can choose some inverted pendulum parameters as well as genetic algorithm's ones.
You can also choose LQR controller in order to compare it with genetic algorithm controller.

Ok, but let's see how does it actually work.

# System control
Let's compare how does the genetic algorithm do its job in comparison to based on mathematical derivations LQR controller.

For that purpose, let's create some simulation scenario:

- Initial conditions:
  - position: 4 m,
  - velocity: 0 m/s,
  - angle: 30 deg,
  - angular velocity: 0 deg/s.
- Reference values:
  - angle: 0 deg,
  - position:
    - from 0 s to 5 s:  0 m,
    - from 5 s to 10 s: 0.8 m,
    - from 10 s to 15 s: -0.5 m.
    
 Let's also choose some system's physical parameters:
- Pendulum:
  - length: 0.8 m,
  - mass: 0.3 kg,
  - friction: 0.05.
- Cart:
  - mass: 0.6 kg,
  - friction: 0.1.
 
Let's check how would this system behave inside a rocket traveling through space with a constant acceleration of g = 9.81 m/s^2.

Ok, everything is set up, so let the battle begin!
Player 1 is...

## LQR based controller
Animation below shows how does the LQR controller manage to stabilize the system:

<p align="center"><img src="anims/anim_LQR.gif" width="750" class="center"/></p>

Looks pretty cool, as we could expect. It bases on mathematical derivations after all. Let's go to Player 2...

## Genetic algorithm based controller
Parameters:
- population size: 50,
- chromosome size: 4,
- gene bounds: (-100.01, 100.01),
- mutation probability: 0.2,
- crossover probability: 0.4,
- crossover rate: 0.2.

Objective function is to minimise an integral of a difference between position and a reference position over time which is then multiplied by squared simulation time.

Why won't we include integral of a pendulum's angle to ensure staying in upright postion?
Because simulation is terminated every time when pendulum's angle exceeds 90 degrees. The quicker it does so,
the less the simulation time. The less the simulation time, the less the value of objective function defined above. The less the value of objective function,
the less likely to survive is the given individual causing pendulum to fall. ~~The less likely the given~~ ok let's finally check out how does this stuff work.

The fittest individual of 100 generations:
<p align="center"><img src="anims/anim_sample_GA.gif" width="750" class="center"/></p>

## Conclusions
Seems like LQR wins the battle as it stabilizes the system quicker. But is it so in every scenario? How about varying friction? Varying masses? The LQR controller concept is about linearization of the inverted pendulum mathematical model. If we add to the model more non-linearities (by changing assumed as constant physical properties mentioned earlier to varying), we will be able to observe how linearized model differs from the "realistic" one.

# Non-linear system control
## Varying pendulum's friction
Let's start with the pendulum friction. We defined its value before as 0.05. But in the reality it is not constant over the whole angular range within which pendulum can rotate. From the realistic example we could expect the friction to be greater at some points and less at the others. Let's arbitrary define its change with respect to the angular position of the pendulum as on the graph showed below:
<p align="center"><img src="plots/alternative scenarios/var pendulums friction/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[93.91;26.27;12.98;16.62]-friction.png" width="500" class="center"/></p>

How does the LQR controller deal with this problem?
<p align="center"><img src="anims/alternative scenarios/var pendulums friction/state[30.0;100.0;3.0;-2.0]-lqr[2000;110]-k[87.24;22.65;10.49;13.05]-animation.gif" width="500" class="center"/></p>

And how does the genetic algorithm based controller do it?
<p align="center"><img src="anims/alternative scenarios/var pendulums friction/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[93.91;26.27;12.98;16.62].gif" width="500" class="center"/></p>

Differences are a little bit difficult to see, so let's put it both on one graph:
<p align="center"><img src="plots/alternative scenarios/var pendulums friction/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[93.91;26.27;12.98;16.62].png" width="500" class="center"/></p>

Looks like this time it was super easy for genetic algorithm to catch up with the LQR controller.
Let's move on with other non-linearity scenarios.

## Varying cart's friction
Again, we arbitrary define friction change over the cart's position
<p align="center"><img src="plots/alternative scenarios/var carts friction/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[84.82;26.07;12.7;15.06]-cart_friction.png" width="500" class="center"/></p>

The LQR controller:
<p align="center"><img src="anims/alternative scenarios/var carts friction/state[30.0;100.0;3.0;-2.0]-lqr[2000;110]-k[93.74;23.0;10.49;15.06]-animation.gif" width="500" class="center"/></p>

The genetic algorithm based controller:
<p align="center"><img src="anims/alternative scenarios/var carts friction/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[84.82;26.07;12.7;15.06].gif" width="500" class="center"/></p>

All together put on one graph:
<p align="center"><img src="plots/alternative scenarios/var carts friction/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[84.82;26.07;12.7;15.06].png" width="500" class="center"/></p>

This is a revolutionary moment for all of those who associate with the Team Genetic Algorithm - the graph above clearly shows it has managed to stabilize the pendulum quicker than the LQR controller.

## Varying pendulum's mass
Finally, let's investigate an impact of varying in time pendulum's mass on control conditions.
<p align="center"><img src="plots/alternative scenarios/var pendulums mass/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[92.49;29.5;9.35;16.26]-pendulum_mass.png" width="500" class="center"/></p>

The LQR controller:
<p align="center"><img src="anims/alternative scenarios/var pendulums mass/state[30.0;100.0;3.0;-2.0]-lqr[2000;110]-k[268.89;69.5;10.49;19.7]-animation.gif" width="500" class="center"/></p>

The genetic algorithm based controller:
<p align="center"><img src="anims/alternative scenarios/var pendulums mass/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[92.49;29.5;9.35;16.26].gif" width="500" class="center"/></p>

All together put on one graph:
<p align="center"><img src="plots/alternative scenarios/var pendulums mass/state[30.0;100.0;3.0;-2.0]-ga[psize20;mprob0.2;crossprob0.4;crossrate0.2]-k[92.49;29.5;9.35;16.26].png" width="500" class="center"/></p>

Here the advantage of genetic algorithm based controller is even more visible.

