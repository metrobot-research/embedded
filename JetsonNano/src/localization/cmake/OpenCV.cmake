find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV_INCLUDE_DIRS")
message(${OpenCV_INCLUDE_DIRS})
#link_directories("/usr/local/lib") # fixme: find these dir.s by "find /usr -name libopencv*" on Linux
#link_directories("/usr/lib/x86_64-linux-gnu") # for Warren's computer and possibly most Linux Ubuntu system
#link_directories("/usr/lib/aarch64-linux-gnu") # for JetsonNano

set(SELF_DEFINED_OpenCV_LIBS "")

file(GLOB_RECURSE OpenCV_LIBS_TEMP "/usr/local/lib/libopencv*")
list(APPEND SELF_DEFINED_OpenCV_LIBS ${OpenCV_LIBS_TEMP})
#file(GLOB_RECURSE OpenCV_LIBS_TEMP "/usr/lib/x86_64-linux-gnu/libopencv*")
#list(APPEND SELF_DEFINED_OpenCV_LIBS ${OpenCV_LIBS_TEMP})
file(GLOB_RECURSE OpenCV_LIBS_TEMP "/usr/lib/aarch64-linux-gnu/libopencv*")
list(APPEND SELF_DEFINED_OpenCV_LIBS ${OpenCV_LIBS_TEMP})

#list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})
#MESSAGE("OpenCV_LIBS:")
#MESSAGE(${OpenCV_LIBS})

list(APPEND ALL_TARGET_LIBRARIES ${SELF_DEFINED_OpenCV_LIBS})
message("SELF_DEFINED_OpenCV_LIBS")
message(${SELF_DEFINED_OpenCV_LIBS})