# Copyright: (C) 2014, Master Thesis
# Authors: Chalikonda Prabhu Kumar
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.12.2)

#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)
#include_directories(/usr/local/include/ /usr/local/)

PROJECT(Parametric_Modelling)


set(Parametric_Modelling_VERSION_MAJOR "0")
set(Parametric_Modelling_VERSION_MINOR "1")
set(Parametric_Modelling_VERSION_PATCH "0")
set(Parametric_Modelling_VERSION "${Parametric_Modelling_VERSION_MAJOR}.${Parametric_Modelling_VERSION_MINOR}.${Parametric_Modelling_VERSION_PATCH}")

find_package(YCM REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YCM_MODULE_PATH})

#find orocos_kdl, with some glue code to support also the use of the old version
find_package(orocos_kdl 1.2.3 REQUIRED QUIET)

# Find kdl_format_io and kdl_codyco
find_package(kdl_codyco 0.1.2 REQUIRED)
find_package(kdl_format_io REQUIRED)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)

# FIND_PACKAGE(YARP)
# FIND_PACKAGE(ICUB)
#FIND_PACKAGE(kdl_codyco)
#FIND_PACKAGE(orocos_kdl)
# FIND_PACKAGE(iDynTree)
#FIND_PACKAGE(orocos-kdl) 
# set(KDL_INCLUDE_DIRS "/usr/include;/usr/local/include")
# FIND_PACKAGE(kdl)

# include_directories("${kdl_INCLUDE_DIRS}")
# FIND_PACKAGE(IDynTree)
FIND_PACKAGE(Gurls)


# add include directories
INCLUDE_DIRECTORIES(${orocos_kdl_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} ${kdl_format_io_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${skinDynLib_INCLUDE_DIRS} ${iDynTree_LIBRARIES} ${ICUB_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS})
#${kdl_codyco_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS})
#${kdl_INCLUDE_DIRS})
# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")  


# # For Hello Icub tutorial
# ADD_EXECUTABLE(Example Example.cpp)
# # we now add the YARP and iCub libraries to our project.
# TARGET_LINK_LIBRARIES(Example 
#                         ${YARP_LIBRARIES} 
#                         ${iDynTree_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${Gurls_LIBRARIES})

#${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Gurls_LIBRARIES})
#${kdl_INCLUDE_DIRS})

# # For Gurls tutorials
# ADD_EXECUTABLE(helloworld helloworld.cpp)
# # we now add the YARP,iCub and other libraries to our project.
# TARGET_LINK_LIBRARIES(helloworld 
#                         ${YARP_LIBRARIES} 
#                         ${iDynTree_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${Gurls_LIBRARIES})
#${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Gurls_LIBRARIES})
#${kdl_INCLUDE_DIRS})


# Parametric modelling with gurls sample
# ADD_EXECUTABLE(Gurls_PM Gurls_PM.cpp)
# # we now add the YARP,iCub and other libraries to our project.
# TARGET_LINK_LIBRARIES(Gurls_PM 
#                         ${YARP_LIBRARIES} 
#                         ${iDynTree_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${orocos_kdl_LIBRARIES} 
#                         ${kdl_codyco_LIBRARIES} 
#                         ${Gurls_LIBRARIES})

# ADD_EXECUTABLE(Data_Modelling Data_Modelling.cpp)
# # we now add the YARP,iCub and other libraries to our project.
# TARGET_LINK_LIBRARIES(Data_Modelling 
#                         ${YARP_LIBRARIES}
#                         ${iDynTree_LIBRARIES}
#                         ${ICUB_LIBRARIES} 
#                         ${orocos_kdl_LIBRARIES} 
#                         ${kdl_codyco_LIBRARIES} 
#                         ctrlLib 
#                         ${orocos_kdl_LIBRARIES} 
# #                                        kdl_codyco 
# #                                         kdl_format_io 
#                         ${iDynTree_LIBRARIES} 
#                         iDyn 
#                         skinDynLib 
# #                                        iDynTree 
#                         ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

# # ${Gurls_LIBRARIES} ${orocos_kdl_LIBRARIES})

# ADD_EXECUTABLE(GURLSloocvgaussian GURLSloocvgaussian.cpp)
# # we now add the YARP,iCub and other libraries to our project.
# TARGET_LINK_LIBRARIES(GURLSloocvgaussian
#                         ${YARP_LIBRARIES} 
#                         ${iDynTree_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${orocos_kdl_LIBRARIES} 
#                         ${kdl_codyco_LIBRARIES} 
#                         ctrlLib 
#                         ${orocos_kdl_LIBRARIES} 
# #                         kdl_codyco 
# #                         kdl_format_io 
#                         ${iDynTree_LIBRARIES} 
#                         iDyn 
#                         skinDynLib 
# #                         iDynTree 
#                         ${YARP_LIBRARIES} 
#                         ${ICUB_LIBRARIES}
#                         ${Gurls_LIBRARIES})

ADD_EXECUTABLE(iCubParis02_simple_analysis 
                iCubParis02_simple_analysis.cpp 
                multitaskSVDLinearEstimator.cpp 
                multitaskRecursiveLinearEstimator.cpp)
# we now add the YARP,iCub and other libraries to our project.
TARGET_LINK_LIBRARIES(iCubParis02_simple_analysis 
                        ${YARP_LIBRARIES} 
                        ${iDynTree_LIBRARIES} 
                        ${ICUB_LIBRARIES} 
                        ${orocos_kdl_LIBRARIES} 
                        ${kdl_codyco_LIBRARIES} 
                        ctrlLib 
                        ${orocos_kdl_LIBRARIES} 
#                                                     kdl_codyco 
#                                                     kdl_format_io 
                        ${iDynTree_LIBRARIES} 
                        iDyn 
                        skinDynLib 
#                                                     iDynTree 
                        ${YARP_LIBRARIES} 
                        ${ICUB_LIBRARIES} 
                        ${Gurls_LIBRARIES})

# ADD_EXECUTABLE(test_for_lambda test_for_lambda.cpp)
# TARGET_LINK_LIBRARIES(test_for_lambda 
#                         ${YARP_LIBRARIES} 
#                         ${iDynTree_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${orocos_kdl_LIBRARIES} 
#                         ${kdl_codyco_LIBRARIES} 
#                         ctrlLib 
#                         ${orocos_kdl_LIBRARIES} 
# #                         kdl_codyco 
# #                         kdl_format_io 
#                         ${iDynTree_LIBRARIES} 
#                         iDyn 
#                         skinDynLib 
# #                         iDynTree 
#                         ${YARP_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${Gurls_LIBRARIES})

# ADD_EXECUTABLE(Data_Model_trail Data_Model_trail.cpp)
# # we now add the YARP,iCub and other libraries to our project.
# TARGET_LINK_LIBRARIES(Data_Model_trail 
#                         ${YARP_LIBRARIES} 
#                         ${iDynTree_LIBRARIES} 
#                         ${ICUB_LIBRARIES} 
#                         ${orocos_kdl_LIBRARIES} 
#                         ${kdl_codyco_LIBRARIES} 
#                         ctrlLib 
#                         ${orocos_kdl_LIBRARIES} 
# #                         kdl_codyco 
# #                         kdl_format_io 
#                         ${iDynTree_LIBRARIES} 
#                         iDyn 
#                         skinDynLib 
# #                         iDynTree 
#                         ${YARP_LIBRARIES} 
#                         ${ICUB_LIBRARIES})