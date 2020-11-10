set(
APPGUI_SRC

Application.cpp
ControllerEditor.cpp
CurveEditor.cpp
Globals.cpp
GLWindow.cpp
InteractionInterface.cpp
InteractiveWorld.cpp
main.cpp
Primitive.cpp
SubGLWindow.cpp
tcltk.cpp
UI.cpp
)

FIND_LIBRARY(GLUT_LIBRARY GLUT )
find_package(TCL REQUIRED)
INCLUDE_DIRECTORIES(${TCL_INCLUDE_PATH} ${TK_INCLUDE_PATH})

set(APP_LIBS core mathlib GLutil ${TCL_LIBRARY} ${TK_LIBRARY})

add_executable(AppGUI ${APPGUI_SRC})
target_link_libraries(AppGUI ${APP_LIBS} ${OPENGL_LIBRARIES} glut)