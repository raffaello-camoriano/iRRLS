
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

// Synchronizes position, velocity, acceleration and force/torque data to form a single Vector for further processing
// Note: Velocity and acceleration are estimated in the callback of the read function, as soon as a new position sample is received

#include <iostream>
#include <iomanip>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/os/Mutex.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;

// A class which handles the incoming data.
// The estimated derivatives are returned at once
// since they are computed within the onRead method.
class dataCollector : public BufferedPort<Bottle>
{
private:
    AWLinEstimator       *linEst;
    AWQuadEstimator      *quadEst;
    
    Vector* PVABuffer;      // pointer to the Vector which contains q, qdot, qdotdot
    Mutex* PVABufferMutex;  // pointer to the Mutex that protects the access to internal buffer containing q, qdot, qdotdot
    
    
    virtual void onRead(Bottle &b)
    {
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);
        
        size_t xsz = b.size();
        Vector x(xsz);

        cout << "Received position bottle: " << b.toString() << endl;
        
        for (int i=0; i < b.size(); i++) {
            x[i] = b.get(i).asDouble();
            cout << "x[" << i <<"] =" << x[i] <<endl;
        }

        // for the estimation the time stamp
        // is required. If not present within the
        // packet, the actual machine time is 
        // attached to it.
        AWPolyElement el(x,info.isValid()?info.getTime():Time::now());
        
        // Protect ON
        PVABufferMutex->lock();
        
        PVABuffer->setSubvector( 0 , x );
        PVABuffer->setSubvector( xsz , linEst->estimate(el) );
        PVABuffer->setSubvector( 2*xsz , quadEst->estimate(el) );
        
        PVABufferMutex->unlock();
        // Protect OFF        
    }

public:
    dataCollector(unsigned int NVel, double DVel, 
                  unsigned int NAcc, double DAcc,
                  Vector* buf,
                  Mutex* bufMut)
    {
        linEst  = new AWLinEstimator(NVel,DVel);
        quadEst = new AWQuadEstimator(NAcc,DAcc);
        PVABuffer = buf;
        PVABufferMutex = bufMut;
    }

    ~dataCollector()
    {
        delete linEst;
        delete quadEst;
    }
};

class Synchronizer: public RFModule
{
private:

    dataCollector        *port_pos;     // Input Vector [ q ]
    BufferedPort<Bottle>  FTport;       // Input Force/torque data    [ F , T ]
    BufferedPort<Vector>  outPort;      // Output vector [ q , qdot, qdotdot, F, T ]
    Port                  rpcPort;      
    Vector*               FTVector;
    size_t                t;            // Size of the F/T vector
    size_t                xsz;          // Size of the F/T vector

public:
    
    Vector PVABuffer;      // Vector which contains q, qdot, qdotdot
    Mutex PVABufferMutex;  // Mutex that protects the access to internal buffer containing q, qdot, qdotdot
    
    virtual bool configure(ResourceFinder &rf)
    {
        // request high resolution scheduling
        Time::turboBoost();

        string portName=rf.check("name",Value("/Synchronizer")).asString().c_str();

        unsigned int NVel=rf.check("lenVel",Value(16)).asInt();
        unsigned int NAcc=rf.check("lenAcc",Value(25)).asInt();

        double DVel=rf.check("thrVel",Value(1.0)).asDouble();
        double DAcc=rf.check("thrAcc",Value(1.0)).asDouble();
        
        t = rf.check("t", Value(6)).asInt();
        xsz = rf.check("xsz", Value(4)).asInt();

        FTVector = new Vector( t , 0.0 );         // Allocate F/T buffer Vector
        
        PVABufferMutex.lock();
        PVABuffer.clear();
        PVABuffer.resize(3*xsz + t , 0.0);
        PVABufferMutex.unlock();

        if (NVel<2)
        {
            cout<<"Warning: lenVel cannot be lower than 2 => N=2 is assumed"<<endl;
            NVel=2;
        }

        if (NAcc<3)
        {
            cout<<"Warning: lenAcc cannot be lower than 3 => N=3 is assumed"<<endl;
            NAcc=3;
        }

        if (DVel<0.0)
        {
            cout<<"Warning: thrVel cannot be lower than 0.0 => D=0.0 is assumed"<<endl;
            DVel=0.0;
        }

        if (DAcc<0.0)
        {
            cout<<"Warning: thrAcc cannot be lower than 0.0 => D=0.0 is assumed"<<endl;
            DAcc=0.0;
        }
        
        // Input positions
        port_pos = new dataCollector(NVel,DVel,NAcc,DAcc, &PVABuffer, &PVABufferMutex);
        port_pos->useCallback();
        port_pos->open((portName + "/pos:i").c_str());

        // Input F/T
        FTport.open((portName + "/ft:i").c_str());
        
        // RPC
        rpcPort.open((portName + "/rpc").c_str());
        attach(rpcPort);
        
        // Output Vector
        outPort.open((portName + "/vec:o").c_str());

        return true;
    }

    virtual bool close()
    {
        port_pos->close();
        FTport.close();
        outPort.close();
        rpcPort.close();

        delete port_pos;
        delete FTVector;

        return true;
    }
    
    bool interruptModule()
    {
        port_pos->interrupt();
        FTport.interrupt();
        outPort.interrupt();
        rpcPort.interrupt();
        
        PVABufferMutex.unlock(); // Unlock the mutex to avoid deadlocks and allow a smooth stop


        return true;
    }    

    virtual double getPeriod()    { return 0.05;  }
    
    virtual bool   updateModule() {
        
        cout << "updateModule " << endl;
        Vector& res = outPort.prepare();
        res.clear();
        res.resize(3*xsz + t);
        
        // Protect ON
        PVABufferMutex.lock();
        res.setSubvector( 0 , PVABuffer );
        PVABufferMutex.unlock();
        // Protect OFF
        
        // Read the most recent F/T        
        Bottle* b = FTport.read();
        for (int i = 0 ; i < b->size() ; i++) {
            (*FTVector)[i] = b->get(i).asDouble();
        }
        
        if (FTVector==0)
        {
            // Skipping...
            return true;
        }
        
        res.setSubvector( 3*xsz , *FTVector );   // WARNING: FTVector must be the most recent reading of the F/T sensor. How to get it in this callback?

        // the outbound packets will carry the same
        // envelope information of the inbound ones.
        if (outPort.getOutputCount()>0)
        {
            //outPort.setEnvelope(info);        // WARNING: missing info. To be implemented
            outPort.write();
        }
        else
            outPort.unprepare();        
        
        return true; 
    }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("Synchronizer_config.ini");
    rf.setDefaultContext("iRRLS");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<<"Options:"<<endl<<endl;
        cout<<"\t--name   name: observer port name (default /velObs)"               <<endl;
        cout<<"\t--lenVel    N: velocity window's max length (default: 16)"         <<endl;
        cout<<"\t--thrVel    D: velocity max deviation threshold (default: 1.0)"    <<endl;
        cout<<"\t--lenAcc    N: acceleration window's max length (default: 25)"     <<endl;
        cout<<"\t--thrAcc    D: acceleration max deviation threshold (default: 1.0)"<<endl;

        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP server not available!"<<endl;
        return -1;
    }

    Synchronizer sync;
    return sync.runModule(rf);
}
