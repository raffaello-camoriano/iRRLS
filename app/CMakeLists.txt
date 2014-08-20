# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname RBDemo)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB lua    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.lua)
message (STATUS ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
message (STATUS ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}) 
message (STATUS ${scripts})                
### create a single target that installs all applications at once
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${lua} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
