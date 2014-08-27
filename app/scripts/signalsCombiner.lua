--
-- Copyright (C) 2014 IIT iCub Facility
-- Author: Raffaello Camoriano
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
-- DESCRIPTION:
-- Combines the position, velocity, acceletarion and F/T signals in a single Vector for further processing
 
-- loading lua-yarp binding library
require("yarp")



PortMonitor.create = function()
    -- Touch information has priority, so it is always accepted
    PortMonitor.setConstraint("true")
    return true;
end

--
-- accept is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Boolean
-- if false is returned, the data will be ignored 
-- and update() will never be called
PortMonitor.accept = function(thing)
    PortMonitor.setEvent("e_touch", 2.0)     -- Set a timout of 2 seconds to return control
    return true
end


