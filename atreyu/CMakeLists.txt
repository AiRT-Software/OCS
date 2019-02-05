
FILE (GLOB SRCS "*.cpp" "include/*.h" "include/interface/*.h")
LIST(REMOVE_ITEM SRCS ${CMAKE_CURRENT_SOURCE_DIR}/grabber.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/grabber.h)  ## old librealsense 1.11
LIST(REMOVE_ITEM SRCS ${CMAKE_CURRENT_SOURCE_DIR}/zr300module.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/zr300module.h)  ## it uses grabber.cpp

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED)

## OpenMP is used for smoothing and normal computations
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    ADD_DEFINITIONS(-DUSINGOPENMP)
    message("OpenMP found!")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(${OpenMP_DEFINITIONS})
else(OPENMP_FOUND)
    message("OpenMP NOT found!")
endif(OPENMP_FOUND)

add_executable (atreyu ${SRCS})

#target_link_libraries (atreyu LINK_PUBLIC pozyx)
target_link_libraries (atreyu LINK_PUBLIC airt)


target_link_libraries(atreyu LINK_PUBLIC libjpeg-turbo)
target_link_libraries(atreyu LINK_PUBLIC zmqpp-static)
target_link_libraries(atreyu LINK_PUBLIC pthread)
target_link_libraries(atreyu -lpcl_common -lpcl_io -lpcl_search -lpcl_filters -lpcl_segmentation -lpcl_recognition)

target_link_libraries(atreyu -lrealsense2) ##librealsense
target_link_libraries(atreyu ${Boost_LIBRARIES} -lboost_system -lboost_thread -lpthread)  ##udp

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/libjpeg-turbo-1.5.90)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/zeromq-4.2.1/include)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/zmqpp/src/zmqpp)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/rapidjson)  ##rapidjson
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/glm-master)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/sml-master/include)  ##non official boost::state machine library
INCLUDE_DIRECTORIES(./include)
INCLUDE_DIRECTORIES(./messages)

INSTALL(TARGETS atreyu DESTINATION local/bin)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/release-files/ini-files/atreyu.ini DESTINATION share/airt)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/release-files/json/pozyx_settings.json DESTINATION share/airt)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/release-files/library/library.txt DESTINATION share/airt/library)
