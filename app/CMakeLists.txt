# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Author: Raffaello Camoriano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname iRRLS)

file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB proj    ${CMAKE_CURRENT_SOURCE_DIR}/conf/proj/*.ini)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB data    ${CMAKE_CURRENT_SOURCE_DIR}/conf/data/*.dat)
file(GLOB lua    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.lua)

message (STATUS ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
message (STATUS ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}) 
message (STATUS ${scripts})                

### create a single target that installs all applications at once
yarp_install(FILES ${templates} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${proj} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/proj)
yarp_install(FILES ${data} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/data)
yarp_install(FILES ${lua} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
