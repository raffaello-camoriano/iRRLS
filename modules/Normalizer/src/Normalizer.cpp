// Performs a Random Features mapping of the input given the projections vector
//
// Reads the projections from the configuration file RFmapper.ini and applies them to the incoming normalized samples
//


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
class Normalizer: public RFModule
{
protected:
    
    // Ports
    BufferedPort<Bottle>      outFeatures;
    BufferedPort<Bottle>      inFeatures;
    Port                      rpcPort;
    
    // Data
    int d;
    int t;
    Bottle maxes;      // Max limits
    Bottle mins;       // Min limits
    
public:
    /************************************************************************/
    Normalizer()
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

        // Set dimensionalities
        d = rf.check("d",Value(12)).asInt();
        t = rf.check("t",Value(6)).asInt();
        
        if (d <= 0  || t <= 0 )
        {
            printf("Error: Inconsistent dimensionalities!\n");
            return false;
        }
        
        // Get fixed limits
        
        maxes = rf.findGroup("LIMITS").findGroup("Max").tail();
        mins = rf.findGroup("LIMITS").findGroup("Min").tail();
        if (maxes.size() != mins.size())
        {
            printf("Error: Inconsistent limits dimensionalities!\n");
            return false;
        }
        
        // Print Configuration
        cout << endl << "-------------------------" << endl;
        cout << "Configuration parameters:" << endl << endl;
        cout << "d = " << d << endl;
        cout << "t = " << t << endl;
        printf("Limits:\n");
        for (int i=0; i<maxes.size(); i++) {
            printf("%d)  " , i);
            printf("Min: %d\t", mins.get(i).asInt());
            printf("Max: %d\n", maxes.get(i).asInt());
        }
        cout << "-------------------------" << endl << endl;
       
        // Open ports
        string fwslash="/";
        inFeatures.open((fwslash+name+"/features:i").c_str());
        printf("inFeatures opened\n");
        outFeatures.open((fwslash+name+"/features:o").c_str());
        printf("outFeatures opened\n");
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
        inFeatures.close();
        printf("inFeatures port closed\n");
        outFeatures.close();
        printf("outFeatures port closed\n");
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

        // Wait for input feature vector
        //cout << "Expecting input feature vector" << endl;
        Bottle *bin = inFeatures.read();    // blocking call
        //cout << "Got it!" << endl << bin->toString() << endl;

        if (bin != 0)
        {
        Bottle& bout = outFeatures.prepare(); // Get a place to store things.
        bout.clear();  // clear is important - b might be a reused object

            // Apply scaling of incoming features
            for (int i = 0 ; i < d+t ; ++i)
            {

                if (i<d)        // Add normalized features
                {
                    if (bin->get(i).asDouble() < mins.get(i).asDouble())
                        bout.add(0.0);
                    else if (bin->get(i).asDouble() > maxes.get(i).asDouble())
                        bout.add(1.0);
                    else
                        bout.add( ( bin->get(i).asDouble() - mins.get(i).asDouble() ) / (maxes.get(i).asDouble() - mins.get(i).asDouble() ) );
                }
                else            // Add labels
                    bout.add(bin->get(i).asDouble());   
            }
            //printf("Sending %s\n", bout.toString().c_str());
            outFeatures.write();
        }

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        // Interrupt any blocking reads on the input port
        inFeatures.interrupt();
        printf("inFeatures port interrupted\n");
        // Interrupt any blocking reads on the output port
        outFeatures.interrupt();
        printf("outFeatures port interrupted\n");

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
    rf.setDefaultConfigFile("Normalizer_config.ini");
    rf.setDefaultContext("iRRLS");
    rf.setDefault("name","Normalizer");
    rf.configure(argc,argv);

    Normalizer mod;
    return mod.runModule(rf);
}
