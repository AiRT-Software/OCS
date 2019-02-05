cmake_minimum_required(VERSION 2.8.3)

set(SourceMessageCodesFile "${CMAKE_SOURCE_DIR}/jiminy/airt_unity/Assets/Connection/Scripts/MessageCodes.cs")
file(STRINGS "${SourceMessageCodesFile}" Contents)

foreach(Line ${Contents})
  if ("${Line}" MATCHES "^.*MAJOR[ ]*=[ ]*([0-9]+),")
    SET(CPACK_PACKAGE_VERSION_MAJOR ${CMAKE_MATCH_1})
  endif()
  if ("${Line}" MATCHES "^.*MINOR[ ]*=[ ]*([0-9]+),")
    SET(CPACK_PACKAGE_VERSION_MINOR ${CMAKE_MATCH_1})
  endif()
  if ("${Line}" MATCHES "^.*PATCH[ ]*=[ ]*([0-9]+)$")
    SET(CPACK_PACKAGE_VERSION_PATCH ${CMAKE_MATCH_1})
  endif()
endforeach()

unset(SourceMessageCodesFile)
unset(Contents)

# CPack configuration
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Paco Abad (fjabad@dsic.upv.es)")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "AiRT OCS (atreyu)")
SET(CPACK_PACKAGE_VENDOR "Instituto de Automática e Informática Industrial (UPV)")
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://www.ai2.upv.es")
SET(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${CMAKE_CURRENT_SOURCE_DIR}/release-files/scripts/postinst)
SET(CPACK_PACKAGE_NAME "airt-project")
SET(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}-${RELEASE}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE})

MESSAGE("Building version airt-system ---${CPACK_PACKAGE_VERSION}---")

INCLUDE(CPack)

PROJECT(airt-project)

# Specific build options
# If you want to build the tests, do cmake -DBUILD_TESTS=ON ..
OPTION(BUILD_TESTS "Compile for testing" OFF)


# Directories for CMake modules
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake})

# Flags for debug
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -D_DEBUG -fsanitize=address -g")
#SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -D_DEBUG -g")

# Install prefix
SET(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

# Flags for gcc
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "-Wall")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# If not specified, we build the project in Release
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)


add_library( libzmq-static STATIC IMPORTED )
set_target_properties( libzmq-static PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmq-static.a )

add_library( restc-cpp STATIC IMPORTED )
set_target_properties( restc-cpp PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/librestc-cpp.a )

add_library( libjpeg-turbo STATIC IMPORTED )
set_target_properties( libjpeg-turbo PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libturbojpeg.a )

add_subdirectory(external-libs)


add_subdirectory(libairt)
add_subdirectory(atreyu)
add_subdirectory(lazarus)


# Prepare MessageCodes.cs for C++

set(SourceMessageCodesFile "${CMAKE_SOURCE_DIR}/jiminy/airt_unity/Assets/Connection/Scripts/MessageCodes.cs")
set(DestinationMessageCodesFile "${CMAKE_SOURCE_DIR}/libairt/include/messages/MessageCodes.h")
file(READ ${SourceMessageCodesFile} Contents)

string(REPLACE "CommandType." "CommandType::" Contents "${Contents}")
string(REPLACE "NotificationType." "NotificationType::" Contents "${Contents}")

file(REMOVE ${DestinationMessageCodesFile})

file(WRITE ${DestinationMessageCodesFile} "/*\n\n  WATCH OUT!!!! \n\nDO NOT MODIFY THIS FILE. DO NOT ADD THIS FILE TO SUBVERSION\n")
file(APPEND ${DestinationMessageCodesFile} "This file is generated automatically from the contents of\n")
file(APPEND ${DestinationMessageCodesFile} "\njiminy/airt_unity/Assets/Connection/Scripts/MessageCodes.cs\n\nAny changes ")
file(APPEND ${DestinationMessageCodesFile} "made to this file will be overriten whenever cmake rebuilds the project.\n")
file(APPEND ${DestinationMessageCodesFile} "If you add a new code to MessageCodes.cs, execute cmake again to rebuild this file\n\n*/\n\n")
file(APPEND ${DestinationMessageCodesFile} "#if defined(__cplusplus)\n")
file(APPEND ${DestinationMessageCodesFile} "#define public\n")
file(APPEND ${DestinationMessageCodesFile} "#define byte uint8_t\n")
file(APPEND ${DestinationMessageCodesFile} "#endif\n")

file(APPEND ${DestinationMessageCodesFile} "${Contents}")

file(APPEND ${DestinationMessageCodesFile} "#if defined(__cplusplus)\n")
file(APPEND ${DestinationMessageCodesFile} "#undef public\n")
file(APPEND ${DestinationMessageCodesFile} "#undef byte\n")
file(APPEND ${DestinationMessageCodesFile} "#endif\n")
