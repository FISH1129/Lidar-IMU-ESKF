<<<<<<< HEAD
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)
=======
#find_package (GeographicLib REQUIRED)
#include_directories(${GeographicLib_INCLUDE_DIRS})
#list(APPEND ALL_TARGET_LIBRARIES ${GeographicLib_LIBRARIES})

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
