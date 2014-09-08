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

//#define CHAR_BIT  8    // Number of bits for a byte


#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <istream>
#include <string>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <istream>
#include <sstream>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Vocab.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/conf/system.h>
#include <yarp/dev/all.h>

YARP_DECLARE_DEVICES(icubmod)

#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;


/************************************************************************/
class RandMotion: public RFModule
{
protected:
    
    // Ports
    Port                        rpcPort;
    
    // Data
    string                      armSide;
    string                      robot;
    Vector                      boxCenterPos;   // Position of the box's center [ xCenter, yCenter, zCenter ]
    Vector                      boxSideSizes;   // Dimensions of the box [ xSize, ySize, zSize ]
    vector<Vector>              boxVertexes;    // vector containing the box vertexes
    
    PolyDriver                  clientCartCtrl;
    ICartesianControl          *icart;   
    
    vector<int> get_bits(unsigned int x)
    {
        vector<int> ret;
        for (unsigned int mask=0x80000000; mask; mask>>=1) {
            ret.push_back((x & mask) ? 1 : 0);
        }
        return ret;
    }
    
    void handToCenter()
    {
        Vector xd(3)/*, od(4)*/; // Target position
        
        xd[0]= boxCenterPos[0] + Rand::scalar( -boxSideSizes[0] , boxSideSizes[0] );
        xd[1]= boxCenterPos[1] + Rand::scalar( -boxSideSizes[1] , boxSideSizes[1] );
        xd[2]= boxCenterPos[2] + Rand::scalar( -boxSideSizes[2] , boxSideSizes[2] );
    
    
         //Target orientation
//         Vector oy(4), ox(4);
// 
//         oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=M_PI;
//         ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=M_PI/2.0;
//         
//         Matrix Ry = iCub::ctrl::axis2dcm(oy);   // from axis/angle to rotation matrix notation
//         Matrix Rx = iCub::ctrl::axis2dcm(ox);
// 
//         Matrix R = Ry*Rx;                 // compose the two rotations keeping the order
//         od = iCub::ctrl::dcm2axis(R);     // from rotation matrix back to the axis/angle notation    
/*
        icart->goToPoseSync(xd,od);   // send request and wait for reply
        icart->waitMotionDone(0.04);*/

        double randDuration = Rand::scalar( 3.0 , 7.0 );

        icart->goToPositionSync(xd , randDuration);   // send request and wait for reply
        icart->waitMotionDone(0.04);
        return;
    }
    
public:
    /************************************************************************/
    RandMotion()
    {
        icart = 0;
    }

    /************************************************************************/    
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

    /************************************************************************/
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
        
        //cout << "Configuration started" << std::endl;
        std::cout << "robot name: " + robot << std::endl;
        
        // Get arm side
        armSide = rf.find("armSide").toString();
        if (armSide!="left" && armSide!="right")
        {
            cout<<"Arm side was not set correctly, setting to left by default."<<endl;
            armSide = "left";
        }
        std::cout << "arm side: " + armSide << std::endl;
        
        // Get bounding box's center (w.r.t. ROOT frame)
        bool isOk = 1;
        boxCenterPos.resize(3);
        
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
        
        std::cout << "Got bounding box center" << std::endl;
        
        // Get bounding box's dimensions (w.r.t. x, y,  z axes)
        boxSideSizes.resize(3);
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
        std::cout << "Got box sides sizes" << std::endl;

        // Check reachability
        
        // Open Cartesian interface
        Property option("(device cartesiancontrollerclient)");
        option.put("remote","/" + robot + "/cartesianController/" + armSide + "_arm");
        option.put("local","/client/" + armSide + "_arm");
        
        if (!clientCartCtrl.open(option))
        {
            return false;
            cout << "Client Cartesian controller openinig failed!" << endl;
        }
        cout << "Client Cartesian controller opened successfully!" << endl;
        
        if (clientCartCtrl.isValid()) {
            clientCartCtrl.view(icart);
            cout << "Cartesian interface initialized successfully!" << endl;
        }
        else
        {
            cout << "Cartesian interface initialization failed!" << endl;
            return false;
        }

        boxVertexes.resize(8);
        // Initialize box vertexes points
        for ( int i = 0 ; i < 8 ; ++i )
        {
            boxVertexes[i] = Vector(3);
            vector<int> bitVec = get_bits( i );
            int vSize = bitVec.size();
            cout << "bitVec #" << i + 1 << " : [ ";
            /* Print path vector to console */
            std::copy(bitVec.begin(), bitVec.end(), std::ostream_iterator<int>(std::cout, " "));
            cout << "]" << endl;

            if (bitVec[vSize-1])
                boxVertexes[i][0] = boxCenterPos[0] + boxSideSizes[0] / 2;
            else
                boxVertexes[i][0] = boxCenterPos[0] - boxSideSizes[0] / 2;
            
            if (bitVec[vSize-2])
                boxVertexes[i][1] = boxCenterPos[1] + boxSideSizes[1] / 2;
            else
                boxVertexes[i][1] = boxCenterPos[1] - boxSideSizes[1] / 2;   
            
            if (bitVec[vSize-3])
                boxVertexes[i][2] = boxCenterPos[2] + boxSideSizes[2] / 2;
            else
                boxVertexes[i][2] = boxCenterPos[2] - boxSideSizes[2] / 2;
        }
        
        cout << "Operational space vertexes:" << endl;
        for (int i = 0 ; i<8 ; ++i)
        {
            cout << "Vertex #" << i+1 << ": [   ";
            for ( int j = 0 ; j < 3 ; ++j )
            {
                cout << boxVertexes[i][j] << "    " ;
            }
            cout << "]" << endl;
        }
        
        // Check reachability for each vertex
        Vector xdhat, odhat, qdhat;     // Response vectors
        for ( int i = 0 ; i < 8 ; ++i )
        {
            isOk = icart->askForPosition(boxVertexes[i],xdhat,odhat,qdhat);
        }

        if ( isOk == 0 )        // Abort!
        {
            printf("Error: At least one box vertex is not reachables!\n");
            return false;
        }     
        
        // Get desired home position of the other arm NOTE: TBI
        
        // Check for potential collisions NOTE: TBI
        
        
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
        // Close device
        clientCartCtrl.close();

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
    YARP_REGISTER_DEVICES(icubmod)
 
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
