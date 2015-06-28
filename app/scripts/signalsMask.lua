--
-- Copyright (C) 2014 IIT iCub Facility
-- Author: Raffaello Camoriano
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
-- FUNCTIONALITIES:
-- Selects only the signals corresponding to the first 4 DOFs of the arm coming from the icub/left_arm/state:o port
-- Subsamples with a given factor 's'. e.g. forwards 1 incoming sample each s samples and ignores the rest.
 
-- How to use the portMonitor

-- In general:
--  $ yarp connect /out /in tcp+recv.portmonitor+script.lua+context.myapp+file.my_lua_script
  
--  'script.lua' tells the carrier to create a port monitor object for Lua.  
--  'context.myapp' tells the resource finder to load the script from the 'myapp' context. 
-- 'file.my_lua_script' indicates 'my_lua_script' should be loaded by monitor object. 
--  'my_lua_script' is located using standard yarp Resource Finder policy. The postfix 
--  (e.g., '.lua') is not necessary.

-- In our case:
-- yarp connect /reachModule/handToBeClosed:o /handCtrl/handToBeClosed:i tcp+recv.portmonitor+script.lua+context.RBDemo+file.handCtrlMonitor
 
-- yarp connect /touchDetector/contact_pos:o /handCtrl/handToBeClosed:i tcp+recv.portmonitor+script.lua+context.RBDemo+file.handCtrlMonitor

-- loading lua-yarp binding library
require("yarp")

-- Parameters
periodFactor = 9
i = 0

PortMonitor.create = function()
    return true;
end

-- 
-- destroy is called when port monitor is destroyed
--
PortMonitor.destroy = function()
end


--
-- accept is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Boolean
-- if false is returned, the data will be ignored 
-- and update() will never be called
--
-- Forwards samples once every 'periodFactor' samples
PortMonitor.accept = function(thing)

    if ( i == 0 ) then
        --print("signalsMask.lua -----> Sample accepted")
        i = periodFactor
        return true
    else
        --print("signalsMask.lua -----> Sample dropped")
        i = i - 1
        return false
    end
end

-- PortMonitor.update
-- ------------------
-- This will be called if the data is accepted by PortMonitor.update(). User can modify and 
-- return it using 'thing' object.
--
-- Forwards the first 4 dofs of the left_arm
PortMonitor.update = function(thing)
    
    -- Create new data of vector type
    th = yarp.Things()
    vec = yarp.Vector()

    vec:push_back(thing:asBottle():get(0):asDouble())
    vec:push_back(thing:asBottle():get(1):asDouble())
    vec:push_back(thing:asBottle():get(2):asDouble())
    vec:push_back(thing:asBottle():get(3):asDouble())
    
    th:setPortWriter(vec)
    return th    
end 
