# Hierarchical Perception Control

It is a pretty simple implementation of Perceptual Control Theory (PCT). 

## Cart Pole Control
Implements Rupert Young hierarchy of Control Unit to control OpenAI Gym Cart Pole.

Basic idea: main perception is Pole Angle must be upright, but it is not possible to directly changed so the hierarchy is:

* to control Pole Angle change Pole Speed
* to control Pole Speed change Cart Position
* to change Cart Position change Cart Speed
* to change Cart Speed apply Action, which this is the only actuator available.
