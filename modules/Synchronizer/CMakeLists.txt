# Copyright: 2014 iCub Facility, Istituto Italiano di Tecnologia
# Author: Raffaello Camoriano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME Synchronizer)
PROJECT(${PROJECTNAME})

file(GLOB source src/*.cpp)
#file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
#source_group("Header Files" FILES ${header})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${source})

target_link_libraries(${PROJECTNAME} ctrlLib
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

##Debug: Print out all variables
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
    # message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

#install(TARGETS ${PROJECTNAME} RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
