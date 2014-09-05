
/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Raffaello Camoriano
 * email: raffaello.camoriano@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Vocab.h>
#include <yarp/math/Math.h>
#include <yarp/conf/system.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/************************************************************************/
class RandMotion: public RFModule
{
protected:
    
    // Ports
    Port                      rpcPort;
    
    // Data
    string      armSide;
    string      robot;
    Vector      boxCenterPos;   // Position of the box's center [ xCenter, yCenter, zCenter ]
    Vector      boxSideSizes;   // Dimensions of the box [ xSize, ySize, zSize ]
    
    handToCenter()
    {
        Vector xd(3), od(4);                            // Set a position in the center in front of the robot
        if (robot== "icubSim")    {
            xd[0]=-0.35 + Rand::scalar(0,0.05); xd[1]=0.05 + Rand::scalar(0,0.04); xd[2]=0.05;}
        else {
            xd[0]=-0.35 + Rand::scalar(0,0.05); xd[1]=0.10 + Rand::scalar(0,0.04); xd[2]=0.10;}
    
    
        Vector oy(4), ox(4);

        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=M_PI;
        ox[0]=1.0; ox[1]=0.0; ox[2]=0; ox[3]=M_PI/2.0;
        
        Matrix Ry = iCub::ctrl::axis2dcm(oy);   // from axis/angle to rotation matrix notation
        Matrix Rx = iCub::ctrl::axis2dcm(ox);

        Matrix R = Ry*Rx;                 // compose the two rotations keeping the order
        //fprintf(stdout,"M = \n[ %s ] \n\n", R.toString().c_str());
        od = iCub::ctrl::dcm2axis(R);     // from rotation matrix back to the axis/angle notation    

        //fprintf(stdout,"Command send to move to %.2f, %.2f, %.2f on the robot frame\n", xd[0], xd[1], xd[2] );

        icart->goToPoseSync(xd,od);   // send request and wait for reply
        icart->waitMotionDone(0.04);
        return;
    }
    
public:
    /************************************************************************/
    RandMotion()
    {
    }

    // rpcPort commands handler
    bool respond(const Bottle &      command,
                 Bottle &      reply)
    {
        // This method is called when a command string is sent via RPC

        // Get command string
        string receivedCmd = command.get(0).asString().c_str();
       
        //int responseCode;   //Will contain Vocab-encoded response

        reply.clear();  // Clear reply bottle
        
        if (receivedCmd == "help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("help");
            reply.addString("quit");
        }
        else if (receivedCmd == "quit")
        {
            reply.addString("Quitting.");
            return false; //note also this
        }
        else
            reply.addString("Invalid command, type [help] for a list of accepted commands.");

        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        // Get robot name
        robot = rf.find("robot").toString();
        if (robot=="")
        {
            cout<<"Robot name was not set correctly, setting to \"icub\" by default."<<endl;
            robot = "icub";
        }
        
        // Get arm side
        armSide = rf.find("armSide").toString();
        if (armSide!="left" && armSide!="right")
        {
            cout<<"Arm side was not set correctly, setting to left by default."<<endl;
            armSide = "left";
        }
        
        // Get bounding box's center (w.r.t. ROOT frame)
        bool isOk = 1;
        if (rf.check("xCenter"))
            boxCenterPos[0] = rf.find("xCenter").asDouble();
        else
            isOk = 0;
        
        if (rf.check("yCenter") && isOk)
            boxCenterPos[1] = rf.find("yCenter").asDouble();
        else
            isOk = 0;
        
        if (rf.check("zCenter") && isOk)
            boxCenterPos[2] = rf.find("zCenter").asDouble();
        else
            isOk = 0;
        
        if ( isOk == 0 )        // Abort!
        {
            printf("Error: Box center not set!\n");
            return false;
        }        
        
        // Get bounding box's dimensions (w.r.t. x, y,  z axes)
        if (rf.check("xSideSize"))
            boxSideSizes[0] = rf.find("xSideSize").asDouble();
        else
            isOk = 0;
        
        if (rf.check("ySideSize") && isOk)
            boxSideSizes[1] = rf.find("ySideSize").asDouble();
        else
            isOk = 0;
        
        if (rf.check("zSideSize") && isOk)
            boxSideSizes[2] = rf.find("zSideSize").asDouble();
        else
            isOk = 0;
        
        if ( isOk == 0 )        // Abort!
        {
            printf("Error: Box dimensions not set!\n");
            return false;
        }        
        
        // Check reachability
        
        // Initialize box vertexes points
        Vector xdhat, odhat, qdhat;
        icart->askForPosition(xd,xdhat,odhat,qdhat);

        // Get desired home position of the other arm
        
        // Check for potential collisions
        
        
        // Print Configuration
        cout << endl << "-------------------------" << endl;
        cout << "Configuration parameters:" << endl << endl;
        cout << "robot = " << robot << endl;
        cout << "armSide = " << armSide << endl;
        cout << "-------------------------" << endl << endl;

        string fwslash="/";
        // Open ports
        rpcPort.open((fwslash+name+"/rpc:i").c_str());
        printf("rpcPort opened\n");

        // Attach rpcPort to the respond() method
        attach(rpcPort);
        
        return true;
    }

    /************************************************************************/
    bool close()
    {        
        // Close ports
        rpcPort.close();
        printf("rpcPort port closed\n");

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        // Period in seconds
        return 0.0;
    }

    /************************************************************************/
    void init()
    {
    }

    /************************************************************************/
    bool updateModule()
    {
        handToCenter();
        
        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        // Interrupt any blocking reads on the rpc port        
        rpcPort.interrupt();
        printf("rpcPort interrupted\n");

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("RandMotion_config.ini");
    rf.setDefaultContext("iRRLS");
    rf.setDefault("name","RandMotion");
    rf.configure(argc,argv);

    RandMotion mod;
    return mod.runModule(rf);
}
