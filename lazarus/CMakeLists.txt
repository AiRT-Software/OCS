
FILE (GLOB SRCS "*.cpp" "include/*.h")

add_executable (lazarus ${SRCS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/libhwcomm/include)

target_link_libraries (lazarus LINK_PUBLIC airt)

target_link_libraries(lazarus LINK_PUBLIC libzmq-static)
target_link_libraries(lazarus LINK_PUBLIC pthread)


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/zmqpp/src/zmqpp)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/external-libs/zeromq-4.2.1/include)
INCLUDE_DIRECTORIES(./include)

# For the serverMessages.h
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/atreyu/messages)

INSTALL(TARGETS lazarus DESTINATION local/bin)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/release-files/ini-files/lazarus.ini DESTINATION share/airt)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/release-files/scripts/lazarus.service DESTINATION /etc/systemd/system)
