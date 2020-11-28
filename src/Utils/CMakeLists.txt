set(
UTILS_SRC
./ImageIO.cpp
./Image.cpp
./Utils.cpp
./BMPIO.cpp
./Timer.cpp
./LogUtil.cpp
./JsonUtil.cpp
)

add_library(util ${UTILS_SRC})

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
target_link_libraries(util jsoncpp mathlib)