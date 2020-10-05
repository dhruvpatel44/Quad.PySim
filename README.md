# Quadcopter Simulation and Control (Quad.PySim)

Looking into Quad Simulation using Python.

This project will serve 2 purposes. The first is to create quadcopter dynamics and outputing the relevant equations of motion. The second purpose is to provide a simple simulation of the quadcopter's dynamics and a simple controller that can handle position control and supports minimum snap trajectory generation.

PyDy for Quadcopter
--------------------

`PyDy`, short for Python Dynamics, is a tool kit made to enable the study of multibody dynamics. At it's core is the `SymPy` mechanics package, which provides an API for building models and generating the symbolic equations of motion for complex multibody systems.

Some `PyDy scripts` express the drone's orientation in the NED frame, and some in the ENU frame.

**NED frame** : The world is oriented in such a way that the X direction in **North**, Y is **East** and Z is **Down**. The drone's orientation in this frame is **front-right-down (frd)**. This is a conventional/classic frame used in aeronautics, and also the frame used for the PX4 multicopter controller. 

**ENU frame** : The world is oreinted in such a way that the X direction is **East**, Y is **North** and Z is **Down**. The drone's orientation in this frame is **front-left-up (flu)**. This frame is widely used for its vizualizing simplicity (z is up). However, it possesses a vizualizing complexity where "pitching up" actually results in a negative pitch angle. 

The other difference in the provided scripts is that some use Euler angles *phi* (*&phi;*), *theta* (*&theta;*), *psi* (*&psi;*) (roll, pitch, yaw) to describes the drone's orientation, while the other scripts uses a quaterninon.

 

