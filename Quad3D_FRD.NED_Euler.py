# ---------------------------------|
# Equations for State derivatives  |
# ---------------------------------|
#                  by Dhruv Patel  |
# ---------------------------------|
#        UNIVERSITY OF CINCINNATI  |
# ---------------------------------|


# we will be using PyDy and SymPy and generate equations for the state derivatives.
# The variable in this script are :

# x, y, z           :   Position of the drone's center of mass in the inertial frame, expressed in the inertial frame.
# xdot, ydot, zdot  :   velocity of the drone's center of mass in the inertial frame, expressed in teh inertial frame.
# phi, theta, psi   :   orientation (roll, pitch, yaw) of the drone in the inertial frame, following the order ZYX (yaw, pitch, roll)
# p, q, r           :   angular velocity of the drone in the interial frame, expressed in the drone's frame.

# body orientation  :   "flu" - front-left-up
# world orientation :   "ENU" - East-North-Up





from sympy import symbols
from sympy.physics.mechanics import *

# Reference frames and Points
# ----------------------------

N = ReferenceFrame('N')     # inertial frame
B = ReferenceFrame('B')     # drone after X (roll) rotation (Final rotation)
C = ReferenceFrame('C')     # drone after Y (pitch) rotation (Second rotation)
D = ReferenceFrame('D')     # drone after Z (yaw) rotation (First rotation)

No = Point('No')
Bcm = Point('Bcm')      # drone's center of mass

M1 = Point('M1')        # Motor 1 is front left, and the rest of the motors are increments CW
                        # (Drone is in X configuration, not +)
M2 = Point('M2')
M3 = Point('M3')
M4 = Point('M4')

# Variables
# ----------

x, y, z, xdot, ydot, zdot = dynamicsymbols('x y z xdot ydot zdot')
phi, theta, psi, p, q, r = dynamicsymbols('phi theta psi p q r')

# First derivatives of the variables
xd, yd, zd, xdotd, ydotd, zdotd = dynamicsymbols('x y z xdot ydot zdot', 1)
phid, thetad, psid, pd, qd, rd = dynamicsymbols('phi theta psi p q r', 1)


# Constants
# ----------

mB, g, dxm, dym, dzm, IBxx, IByy, IBzz = symbols('mB g dxm dym dzm IBxx IByy IBzz')
ThrM1, ThrM2, ThrM3, ThrM4, TorM1, TorM2, TorM3, TorM4 = symbols('ThrM1 ThrM2 ThrM3 ThrM4 TorM1 TorM2 TorM3 TorM4')

# Rotation ZYX Body
# ------------------
# Rframe.orient() : generates a direction cosine matrix and its transpose which defines
#              ...  the orientation of D relative to N and vice versa.
#              ...  Once orient is called, dcm() outputs the appropriate direction cosine matrix.

D.orient(N, 'Axis', [psi, N.z])
C.orient(D, 'Axis', [theta, D.y])
B.orient(C, 'Axis', [phi, C.x])

# Origin
# -------
# set_vel(frame, value) : Sets the velocity Vector of the Point in a ReferenceFrame.
# It will set the velocity of a point to a certain value, for a certain Ref. Frame.

No.set_vel(N, 0)        # sets velocity of point (No) to 0, in (N) ref. frame.

# Translation
# ------------
# set_pos(otherpoint, value) :  it is used to set the position of the point w.r.t. another point.
# pos_from(otherpoint)       :  it returns a Vector distance between the point and the other point.

Bcm.set_pos(No, x*N.x + y*N.y + z*N.z)  # setting pos of center mass (Bcm) w.r.t Inertial frame origin (No).
Bcm.set_vel(N, Bcm.pos_from(No).dt(N))  # setting the velocity of center mass (Bcm) in (N) frame, by calculating the distance from origin and dividing it by time.

# Motor placement
# ----------------
# M1 is front left, then clockwise numbering
# (dzm) is positive for motors above center of mass

M1.set_pos(Bcm, dxm*B.x + dym*B.y + dzm*B.z)
M2.set_pos(Bcm, dxm*B.x - dym*B.y + dzm*B.z)
M3.set_pos(Bcm, - dxm*B.x - dym*B.y + dzm*B.z)
M4.set_pos(Bcm, - dxm*B.x + dym*B.y + dzm*B.z)

# v2pt_theory(otherpoint, outframe, fixedframe) : it sets the velocity of the point with the 2-point theory.
# {M1} and {Bcm}, both points are fixed in body-frame {B}, while rotating in Inertial-frame {N}.

M1.v2pt_theory(Bcm, N, B)       # will give {M1} point's velocity in Inertial-frame {N}
M2.v2pt_theory(Bcm, N, B)
M3.v2pt_theory(Bcm, N, B)
M4.v2pt_theory(Bcm, N, B)

# Inertia Dyadic
# ---------------
# inertia(frame, ixx, iyy, izz, ixy=0, iyz=0, izx=0) : it is a simple way to create inertia Dyadic object.
# ... frame : the frame in which inertia is defined. (usually the body frame)

IB = inertia(B, IBxx, IByy, IBzz)

# Create Bodies
# --------------
# RigidBody(name, masscenter, frame, mass, inertia) : it creates an idealized rigid body.
# ... name          : the body's name
# ... masscenter    : point which represents center of mass
# ... frame         : the Ref. frame in which the rigid body is fixed in. (Body-frame)
# ... inertia       : ((Dyadic, point)) : inertia of the body about a point.

BodyB = RigidBody('BodyB', Bcm, B, mB, (IB, Bcm))
BodyList = [BodyB]

# Forces and Torques
# -------------------

Grav_Force = (Bcm, -mB*g*N.z)

FM1 = (M1, ThrM1*B.z)
FM2 = (M2, ThrM2*B.z)
FM3 = (M3, ThrM3*B.z)
FM4 = (M4, ThrM4*B.z)

TM1 = (M1, TorM1*B.z)
TM2 = (M1, TorM2*B.z)
TM3 = (M1, TorM3*B.z)
TM4 = (M1, TorM4*B.z)

ForceList = [Grav_Force, FM1, FM2, FM3, FM4, TM1, TM2, TM3, TM4]

# Kinematic Differential Equations
# ---------------------------------

kd = [xdot - xd, ydot - yd, zdot - zd, p - dot(B.ang_vel_in(N), B.x), q - dot(B.ang_vel_in(N), B.y), r - dot(B.ang_vel_in(N), B.z)]

# Kane's Method
# --------------

KM = KanesMethod(N, q_ind=[x, y, z, phi, theta, psi],
                    u_ind=[xdot, ydot, zdot, p, q, r],
                    kd_eqs=kd)

(fr, frstar) = KM.kanes_equations(BodyList, ForceList)


# Equations of Motion
# --------------------

MM = KM.mass_matrix_full
kdd = KM.kindiffdict()
rhs = KM.forcing_full
rhs = rhs.subs(kdd)

MM.simplify()
print(f'| Mass Matrix |')
print(f'------------ \n')
mprint(MM)
print()

rhs.simplify()
print(f'| Right Hand Side |')
print(f'---------------- \n')
mprint(rhs)
print()

# So, MM*x = rhs, where x is the State Derivatives
# Solve for x

stateDot = MM.inv()*rhs
print(f'| State Derivatives |')
print(f'------------------ \n')
mprint(stateDot)
print()
