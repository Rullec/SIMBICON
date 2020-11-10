set(
PHYSICS_SRC

AbstractRBEngine.cpp
ArticulatedFigure.cpp
ArticulatedRigidBody.cpp
BallInSocketJoint.cpp
BoxCDP.cpp
CapsuleCDP.cpp
CollisionDetectionPrimitive.cpp
HingeJoint.cpp
Joint.cpp
ODEWorld.cpp
PlaneCDP.cpp
PreCollisionQuery.cpp
RBProperties.cpp
RBState.cpp
RBUtils.cpp
RigidBody.cpp
SphereCDP.cpp
UniversalJoint.cpp
)

add_library(physics ${PHYSICS_SRC})
target_link_libraries(physics mathlib ode util GLutil)