find_package (Protobuf REQUIRED)
#set(Protobuf_PREFIX_PATH
#    "/usr/local/include"
#   "/usr/local/lib"
#   "/usr/local/bin"
#)
#list(APPEND CMAKE_PREFIX_PATH "${Protobuf_PREFIX_PATH}")
#find_package(Protobuf  REQUIRED PATH /usr/local)

INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
list(APPEND ALL_TARGET_LIBRARIES ${PROTOBUF_LIBRARIES})                                                                                                                                                                                                          
