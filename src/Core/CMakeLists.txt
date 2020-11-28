set(
CORE_SRC

BalanceFeedback.cpp
BaseControlFramework.cpp
Character.cpp
Controller.cpp
ConUtils.cpp
PoseController.cpp
SimBiConFramework.cpp
SimBiConState.cpp
CustomSimBiConState.cpp
SimBiController.cpp
CustomSimBiController.cpp
SimGlobals.cpp
Trajectory.cpp
SimTraj.cpp
CustomSimTraj.cpp
)
add_library(core ${CORE_SRC})
target_link_libraries(core mathlib physics)