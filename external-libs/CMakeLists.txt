
add_subdirectory(cpptoml)
#add_subdirectory(zeromq-4.2.1)
set(ZEROMQ_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external-libs/zeromq-4.2.1/include)
set(ZEROMQ_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(zmqpp)
