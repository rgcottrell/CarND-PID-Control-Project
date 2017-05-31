# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID Controller

A PID controller provides a simple feedback mechanism to reduce the error in
a system. In this case, the error was a measurement of how far off center a
self-driving car was.

The P, or proportional, component of the controller adjusts the system in
proportion to the current error. In this case, the further away the car was
from the center of the track, the greater the steering angle chosen. The P
term is prone to overshooting the target.

The D, or derivate, component of the controller adjusts the system based on
the derivative, or rate of change, of the error. This provides a dampening
effect that can help smooth out the overshooting introduced by the P term.

The I, or integral, component of the controller adjust the system based on
the running sum of the error. This helps combat systematic bias in the
system such as a poorly aligned steering wheel. In this simulation, however,
there was no detected bias and the best results were found when the I term
was set to 0.

## Tuning

The PID values were chosen based on a mix of manual tuning as well as
optimization through coordinate ascent using the Twiddle algorithm.

Initially, values were chosen manually to get an intuitive feel for how
different values would influence the responsiveness of the car. This also
provided a reasonable estimate of the deltas that should be used by the
Twiddle algorithm.

Next, the Twiddle algorithm was used to optimize parameters to reduce the
error in the initial portion of the track. After a certain number of steps
were evaluated, the simulation was restarted to test a new set of PID
parameters.

Finally, the values were manually adjusted to work better on the track.
For example, the trained values worked well for straight potions of the
track but still overcompensated in some areas, so the D parameter was
increased to provide more dampening.

## Reflection

The Twiddle algorithm proved to be effective at systemically experimenting
with the parameters to converge on a set that was able to effectively
navigate the track. However, there were some limitations with the way the
algorithm was implemented.

Most of the training was done on the mostly straight initial stretch of the
track. Because of this, the values that the algorithm converged to proved
insufficient to handle the curved parts of the track.

The training did not take into account the speed of the car. At higher speeds,
it's easier for the car to overcompensate. A more effective training model
would include adjustments for speed as well as the ability to brake or
accelerate to maneuver more effectively.

The training was limited to a fixed number of update steps. While this made
the errors comparable over successive runs, it also prevented training on
different parts of the track. A better approach might be to vary the length
of each trial based on how well the car is navigating the track and then
find a way to compare the results of training sessions of different lengths.

Finally, the training was all done in real time, which resulted in long
training times. It would be helpful if the simulator were allowed to run
faster to allow for more training in shorter time.