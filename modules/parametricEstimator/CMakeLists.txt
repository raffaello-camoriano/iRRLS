# Copyright: 2014 iCub Facility, Istituto Italiano di Tecnologia
# Author: Raffaello Camoriano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

cmake_minimum_required(VERSION 2.8.12.2)

SET(PROJECTNAME parametricEstimator)
PROJECT(${PROJECTNAME})


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
find_package(Gurls)

# add include directories
INCLUDE_DIRECTORIES(${orocos_kdl_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} ${kdl_format_io_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${skinDynLib_INCLUDE_DIRS} ${iDynTree_LIBRARIES} ${ICUB_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")  

file(GLOB source src/*.cpp)
#file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
#source_group("Header Files" FILES ${header})

add_definitions(${Gurls_DEFINITIONS})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS})


ADD_EXECUTABLE(${PROJECTNAME}  ${source})
                
# We now add the YARP,iCub and other libraries to our project.
TARGET_LINK_LIBRARIES(${PROJECTNAME}
                        ${YARP_LIBRARIES} 
                        ${iDynTree_LIBRARIES} 
                        ${ICUB_LIBRARIES} 
                        ${orocos_kdl_LIBRARIES} 
                        ${kdl_codyco_LIBRARIES} 
                        ctrlLib 
                        ${orocos_kdl_LIBRARIES} 
                        ${iDynTree_LIBRARIES} 
                        iDyn 
                        skinDynLib 
                        ${YARP_LIBRARIES} 
                        ${ICUB_LIBRARIES} 
                        ${Gurls_LIBRARIES}
                        ${Gurls++_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

##Debug: Print out all variables
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
    # message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

#install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
