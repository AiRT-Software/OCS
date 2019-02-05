cmake_minimum_required(VERSION 2.8)

# AIRT Common library

find_package(Boost REQUIRED)

FILE (GLOB SRCS "*.cpp" "include/*.h")

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/spdlog/include)

IF(WIN32)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/dirent-1.20.1/include)
ENDIF(WIN32)

ADD_LIBRARY(airt ${SRCS})

target_include_directories(airt INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(airt INTERFACE ${CMAKE_SOURCE_DIR}/external-libs/spdlog/include)
target_include_directories(airt INTERFACE ${CMAKE_SOURCE_DIR}/external-libs/cpptoml/include)

target_link_libraries(airt ${Boost_LIBRARIES} -lboost_system -lboost_filesystem)  
target_link_libraries(airt LINK_PUBLIC cpptoml)

target_link_libraries(airt LINK_PUBLIC zmqpp-static)
target_link_libraries(airt LINK_PUBLIC libzmq-static)
target_link_libraries(airt LINK_PUBLIC pthread)


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/zeromq-4.2.1/include)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/zmqpp/src/zmqpp)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/glm-master)
INCLUDE_DIRECTORIES(./include)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/rapidjson)  ##rapidjson
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/restc-cpp-master/include)  ##restc-cpp
target_link_libraries (airt LINK_PUBLIC restc-cpp -lboost_coroutine -lboost_log)  ##restc-cpp

#INSTALL(TARGETS airt DESTINATION lib)

