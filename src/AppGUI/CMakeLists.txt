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

find_package(GLUT REQUIRED )
set(APP_LIBS core mathlib GLutil ${TCL_LIBRARY} ${TK_LIBRARY})

add_executable(AppGUI ${APPGUI_SRC})
target_link_libraries(AppGUI ${APP_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})