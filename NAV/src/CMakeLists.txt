
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)

pkg_search_module(GATTLIB REQUIRED gattlib)
pkg_search_module(PCRE REQUIRED libpcre)

set(BT_SRCS bt.cpp)
set(MQTT_SRCS mqtt.cpp)

add_executable(bt2mqtt bt2mqtt.cpp ${BT_SRCS} ${MQTT_SRCS})
target_link_libraries(bt2mqtt ${GATTLIB_LIBRARIES} ${GATTLIB_LDFLAGS} ${PCRE_LIBRARIES} pthread)
target_link_libraries (bt2mqtt  mosquittopp)