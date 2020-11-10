set(
GLUTILS_SRC
GLCamera.cpp
GLMesh.cpp
GLTexture.cpp
GLUtils.cpp
OBJReader.cpp
)

add_library(GLutil ${GLUTILS_SRC})
target_link_libraries(GLutil  ${OPENGL_LIBRARIES})