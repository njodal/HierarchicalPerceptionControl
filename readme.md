# Hierarchical Perception Control

It is a pretty simple implementation of Perceptual Control Theory (PCT). 

## Cart Pole Control
Implements Rupert Young hierarchy of Control Unit to control OpenAI Gym Cart Pole (https://perceptualrobots.github.io/pct/examples.html).

Basic idea: main perception is Pole Angle must be upright, but it is not possible to directly change it given the only actuator available is Action (which applies a force to the cart), so the following hierarchy is necessary:

* to control Pole Angle change Pole Speed
* to control Pole Speed change Cart Position
* to change Cart Position change Cart Speed
* to change Cart Speed apply Action

To experiment with different gains and pole angle references, create new examples in CarPole.test.

Run CartPole.py to test it.
