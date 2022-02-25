find_package(PCL 1.7 REQUIRED)
<<<<<<< HEAD
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
=======
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3

include_directories(${PCL_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${PCL_LIBRARIES})