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
set(APP_LIBS core mathlib)
add_executable(AppGUI ${APPGUI_SRC})
target_link_libraries(AppGUI ${APP_LIBS})