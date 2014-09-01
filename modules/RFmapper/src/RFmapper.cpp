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

/** 
\defgroup RFmapper
 
Example of...

Copyright (C) 2014 RobotCub Consortium
 
Author: Raffaello Camoriano

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

\section intro_sec Description 
A module that...

\author Raffaello Camoriano
*/ 

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>
#include <vector>
#include <istream>
#include <string>
#include <sstream>

#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/conf/system.h>
#include <iCub/perception/models.h>

//YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;

/************************************************************************/
// load_matrix function
/************************************************************************/

// load matrix from an ascii text file.
void load_matrix(std::istream* is,
        Matrix& matrix,
        const std::string& delim = " ")
{
    using namespace std;

    string      line;
    string      strnum;

    long unsigned int rowidx =  0;
    long int colidx =  -1;
    
    // parse line by line
    while (getline(*is, line))
    {
        for (string::const_iterator i = line.begin(); i != line.end(); i++)
        {
            
            // If i is not a delim, then append it to strnum
            if (delim.find(*i) == string::npos)
            {
                strnum += *i;
                if(i+1 != line.end())
                {
                    
                    continue;
                }
            }
            
            // if strnum is still empty, it means the previous char is also a
            // delim (several delims appear together). Ignore this char.
            if (strnum.empty())
                continue;

            // If we reach here, we got a number. Convert it to double.
            double number;

            istringstream(strnum) >> number;
            ++colidx;
            matrix[rowidx][colidx] = number;
            
            strnum.clear();            
        }        
        ++rowidx;
        colidx = -1;
    }
}



/************************************************************************/
class RFmapper: public RFModule
{
protected:
    
    // Ports
    BufferedPort<Vector>      outFeatures;
    BufferedPort<Vector>      inFeatures;
    Port                      rpcPort;
    
    // Data
    int d;
    int t;
    int numRF;
    string projFName;   // File name of the projections matrix
    Matrix projMat;    // Pointer to the [numRF x d]-dimensional list of projections
    int mappingType;

    Vector vin;
    Vector vout;
    Vector xin;
    Vector xout;
    
public:
    /************************************************************************/
    RFmapper()
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

        Property config;
        config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            printf("Error: group general is missing!\n");
            return false;
        }

        // Set dimensionalities
        d = rf.findGroup("general").check("d",Value(0)).asInt();
        t = rf.findGroup("general").check("t",Value(0)).asInt();
        numRF = rf.findGroup("general").check("numRF",Value(0)).asInt();
        projMat.resize(numRF,d);      // Initialize projections matrix
            
        if (d <= 0 || t <= 0 || numRF <= 0)
        {
            printf("Error: Inconsistent dimensionalities!\n");
            return false;
        }

        // Load precomputed projections from the specified file
        string projFName = rf.findGroup("general").find("proj").toString();
        if (projFName=="")
        {
            cout<<"Sorry no projections were found, check config parameters"<<endl;
            return -1;
        }
        projFName = rf.getContextPath() + "/proj/" + projFName;
        cout << "Using projections file: " << projFName.c_str() << endl;

        ifstream* ifs = new ifstream;   //WARNING: Deallocate!

        cout << "Trying to open ifstream..." << endl;        
        ifs->open(projFName.c_str(), std::ifstream::in);
        cout << "ifstream opened..." << endl;
        load_matrix(ifs, projMat, " ");
        cout << "Projections matrix loaded. Size: " << projMat.rows() << " x " << projMat.cols() << endl;
        
        if (projMat.rows() != numRF || projMat.cols() != d )
        {
            printf("Error: Inconsistent dimensionalities!\n");
            return false;
        }
        
        // Set mapping type
        mappingType = rf.findGroup("general").check("mappingType",Value(1)).asInt();
    
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
        
        // Wait for incoming sample
        Vector *vin = inFeatures.read();    // blocking call

        if (vin == 0)
        {
            printf("Error: Read failed!\n");
            return false;            
        }
        xin = vin->subVector( 0 , d );   // Select inputs only
        
        // Apply random projections to incoming features
        
        if (mappingType == 1)
        {
            Vector wx(numRF);
            
            wx = projMat * xin;
            
            Vector sinwx(numRF);
            Vector coswx(numRF);
            
            for ( int i = 0 ; i < numRF ; ++i )
            {
                sinwx[i] = sin(wx[i]);
                coswx[i] = cos(wx[i]);
            }
            
            // Send output features
            Vector &xout = outFeatures.prepare();
            xout.clear(); //important, objects get recycled
            
            xout.setSubvector(0, sinwx);
            xout.setSubvector(numRF, coswx);        
            
            outFeatures.write();
        }
        else
        {
            printf("Error: Mapping type not available!\n");
            return false;  
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

    //YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("RFmapper_config.ini");
    rf.setDefaultContext("iRRLS");
    rf.setDefault("name","RFmapper");
    rf.configure(argc,argv);

    RFmapper mod;
    return mod.runModule(rf);
}
