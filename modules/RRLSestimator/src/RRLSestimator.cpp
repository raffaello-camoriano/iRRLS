
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
#include <sstream>
#include <iomanip>
#include <string>
#include <yarp/os/Time.h>

#include "gurls++/recrlswrapperchol.h"
#include "gurls++/rlsprimal.h"
#include "gurls++/primal.h"

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Vocab.h>
#include <yarp/math/Math.h>
#include <yarp/conf/system.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace gurls;

typedef double T;

/************************************************************************/
class RRLSestimator: public RFModule
{
protected:
    
    // Ports
    BufferedPort<Bottle>      inVec;
    BufferedPort<Bottle>      pred;
    BufferedPort<Bottle>      perf;
    Port                      rpcPort;
    
    // Data
    bool verbose;
    int d;
    int t;
    string perfType;
    int savedPerfNum;           // Number of saved performance measurements
    int numPred;                // Number of saved predictions to peform before module closure
    int pretrain;               // Preliminary batch training required
    string pretrainFile;        // Preliminary batch training file
    int n_pretr;                // Number of pretraining samples
    string pretr_type;          // Pretraining type: 'fromFile' or 'fromStream'
    long unsigned int updateCount;      // Prediciton number counter
    int experimentCount;
    
    gMat2D<T> trainSet;    
    gMat2D<T> Xtr;    
    gMat2D<T> ytr;    
    RecursiveRLSCholUpdateWrapper<T> estimator;
    gMat2D<T> varCols;          // Matrix containing the column-wise variances computed on the training set
    
    gMat2D<T> error;
    gMat2D<T> storedError;      // Contains the first numErr computed errors

public:
    /************************************************************************/
    RRLSestimator() : estimator("recursiveRLSChol"), updateCount(0)
    {
    }

    // rpcPort commands handler
    bool respond(const Bottle &      command,
                 Bottle &      reply)
    {
        // This method is called when a command string is sent via RPC

        // Get command string
        string receivedCmd = command.get(0).asString().c_str();
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
        
        // Set verbosity
        verbose = rf.check("verbose",Value(0)).asInt();
               
        // Set dimensionalities
        d = rf.check("d",Value(0)).asInt();
        t = rf.check("t",Value(0)).asInt();
        
        if (d <= 0 || t <= 0 )
        {
            printf("Error: Inconsistent dimensionalities!\n");
            return false;
        }
        
        // Set perf type
        perfType = rf.check("perf",Value("RMSE")).asString();
        
        if ( perfType != "MSE" || perfType != "RMSE" || perfType != "nMSE" )
        {
            printf("Error: Inconsistent performance measure! Set to RMSE.\n");
            perfType = "RMSE";
        }
        
        // Set number of saved performance measurements
        numPred  = rf.check("numPred",Value("-1")).asInt();
        
        // Set number of saved performance measurements
        savedPerfNum = rf.check("savedPerfNum",Value("0")).asInt();
        if (savedPerfNum > numPred)
        {
            savedPerfNum = numPred;
            cout << "Warning: savedPerfNum > numPred, setting savedPerfNum = numPred" << endl;
        }
        
        //experimentCount = rf.check("experimentCount",Value("0")).asInt();
        experimentCount = rf.find("experimentCount").asInt();
        
        // Set preliminary batch training preferences
        pretrain = rf.check("pretrain",Value("0")).asInt();
        
        if ( pretrain == 1 )
        {            
            // Set preliminary batch training file path
            pretrainFile = rf.check("pretrainFile",Value("icubdyn.dat")).asString();
            
            n_pretr = rf.check("n_pretr",Value("2")).asInt();
            
            pretr_type = rf.check("pretr_type" , Value("fromStream")).asString();
            if ((pretr_type != "fromFile") && (pretr_type != "fromFile"))
                pretr_type == "fromFile";
        }
        
        // Print Configuration
        cout << endl << "-------------------------" << endl;
        cout << "Configuration parameters:" << endl << endl;
        cout << "experimentCount = " << experimentCount << endl;
        cout << "d = " << d << endl;
        cout << "t = " << t << endl;
        cout << "perf = " << perfType << endl;
        if ( pretrain == 1 )
        {
            printf("Pretraining requested\n");
            printf("Pretraining type: %s\n", pretr_type.c_str());
            if (pretr_type == "fromFile")
                printf("Pretraining file name set to: %s\n", pretrainFile.c_str());
            printf("Number of pretraining samples: %d\n", n_pretr);
        }
        cout << "-------------------------" << endl << endl;
       
        // Open ports
    
        string fwslash="/";
        inVec.open((fwslash+name+"/vec:i").c_str());
        printf("inVec opened\n");
        
        pred.open((fwslash+name+"/pred:o").c_str());
        printf("pred opened\n");
        
        perf.open((fwslash+name+"/perf:o").c_str());
        printf("perf opened\n");
        
        rpcPort.open((fwslash+name+"/rpc:i").c_str());
        printf("rpcPort opened\n");

        // Attach rpcPort to the respond() method
        attach(rpcPort);

        // Initialize random number generator
        srand(static_cast<unsigned int>(time(NULL)));

        // Initialize error structures
        error.resize(1,t);
        error = gMat2D<T>::zeros(1, t);          //
        
        if (savedPerfNum > 0)
        {
            storedError.resize(savedPerfNum,t);
            storedError = gMat2D<T>::zeros(savedPerfNum, t);          //MSE            
        }

        updateCount = 0;
        
        //------------------------------------------
        //         Pre-training
        //------------------------------------------

        if ( pretrain == 1 )
        {
            if ( pretr_type == "fromFile" )
            {
                //------------------------------------------
                //         Pre-training from file
                //------------------------------------------
                string trainFilePath = rf.getContextPath() + "/data/" + pretrainFile;
                
                try
                {
                    // Load data files
                    cout << "Loading data file..." << endl;
                    trainSet.readCSV(trainFilePath);

                    cout << "File " + trainFilePath + " successfully read!" << endl;
                    cout << "trainSet: " << trainSet << endl;
                    cout << "n_pretr = " << n_pretr << endl;
                    cout << "d = " << d << endl;

                    //WARNING: Add matrix dimensionality check!

                    // Resize Xtr
                    Xtr.resize( n_pretr , d );
                    
                    // Initialize Xtr
                    //Xtr.submatrix(trainSet , n_pretr , d);
                    Xtr.submatrix(trainSet , 0 , 0);
                    cout << "Xtr initialized!" << endl << Xtr << endl;

                    // Resize ytr
                    ytr.resize( n_pretr , t );
                    cout << "ytr resized!" << endl;
                    
                    // Initialize ytr
                    gVec<T> tmpCol(trainSet.rows());
                    cout << "tmpCol" << tmpCol << endl;
                    for ( int i = 0 ; i < t ; ++i )
                    {
                        cout << "trainSet(d + i): " << trainSet(d + i) << endl;
                        tmpCol = trainSet(d + i);
                        gVec<T> tmpCol1(n_pretr);

                        //cout << tmpCol.subvec( (unsigned int) n_pretr ,  (unsigned int) 0);       // WARNING: Fixed in latest GURLS version

                        gVec<T> locs(n_pretr);
                        for (int j = 0 ; j < n_pretr ; ++j)
                            locs[j] = j;
                        cout << "locs" << locs << endl;
                        gVec<T>& tmpCol2 = tmpCol.copyLocations(locs);
                        cout << "tmpCol2" << tmpCol2 << endl;
                    
                        //tmpCol1 = tmpCol.subvec( (unsigned int) n_pretr );
                        //cout << "tmpCol1: " << tmpCol1 << endl;
                        ytr.setColumn( tmpCol2 , (long unsigned int) i);
                    }
                    cout << "ytr initialized!" << endl;

                    // Compute variance for each output on the training set
                    gMat2D<T> varCols = gMat2D<T>::zeros(1,t);
                    gVec<T>* sumCols_v = ytr.sum(COLUMNWISE);          // Vector containing the column-wise sum
                    gMat2D<T> meanCols(sumCols_v->getData(), 1, t, 1); // Matrix containing the column-wise sum
                    meanCols /= n_pretr;        // Matrix containing the column-wise mean
                    
                    if (verbose) cout << "Mean of the output columns: " << endl << meanCols << endl;
                    
                    for (int i = 0; i < n_pretr; i++)
                    {
                        gMat2D<T> ytri(ytr[i].getData(), 1, t, 1);
                        varCols += (ytri - meanCols) * (ytri - meanCols); // NOTE: Temporary assignment
                    }
                    varCols /= n_pretr;     // Compute variance
                    if (verbose) cout << "Variance of the output columns: " << endl << varCols << endl;

                    // Initialize model
                    cout << "Batch pretraining the RLS model with " << n_pretr << " samples." << endl;
                    estimator.train(Xtr, ytr);
                }
                
                catch (gException& e)
                {
                    cout << e.getMessage() << endl;
                    return false;   // Terminate program. NOTE: May be worth to set up specific error return values
                }
            }
            else if ( pretr_type == "fromStream" )
            {
                //------------------------------------------
                //         Pre-training from stream
                //------------------------------------------
                
                try
                {
                    cout << "Pretraining from stream started. Listening on port vec:i." << n_pretr << " samples expected." << endl;

                    // Resize Xtr
                    Xtr.resize( n_pretr , d );
                    
                    // Resize ytr
                    ytr.resize( n_pretr , t );
                    
                    // Initialize Xtr
                    for (int j = 0 ; j < n_pretr ; ++j)
                    {
                        // Wait for input feature vector
                        if(verbose) cout << "Expecting input vector # " << j+1 << endl;
                        
                        Bottle *bin = inVec.read();    // blocking call
                        
                        if (bin != 0)
                        {
                            if(verbose) cout << "Got it!" << endl << bin->toString() << endl;

                            //Store the received sample in gMat2D format for it to be compatible with gurls++
                            for (int i = 0 ; i < bin->size() ; ++i)
                            {
                                if ( i < d )
                                {
                                    Xtr(j,i) = bin->get(i).asDouble();
                                }
                                else if ( (i>=d) && (i<d+t) )
                                {
                                    ytr(j, i - d ) = bin->get(i).asDouble();
                                }
                            }
                            if(verbose) cout << "Xtr[j]:" << endl << Xtr[j] << endl << "ytr[j]:" << endl << ytr[j] << endl;
                        }
                        else
                            --j;        // WARNING: bug while closing with ctrl-c
                    }
                    
                    cout << "Xtr initialized!" << endl;
                    cout << "ytr initialized!" << endl;
                        
                    // Compute variance for each output on the training set
                    gMat2D<T> varCols = gMat2D<T>::zeros(1,t);
                    gVec<T>* sumCols_v = ytr.sum(COLUMNWISE);          // Vector containing the column-wise sum
                    gMat2D<T> meanCols(sumCols_v->getData(), 1, t, 1); // Matrix containing the column-wise sum
                    meanCols /= n_pretr;        // Matrix containing the column-wise mean
                    
                    if (verbose) cout << "Mean of the output columns: " << endl << meanCols << endl;
                    
                    for (int i = 0; i < n_pretr; i++)
                    {
                        gMat2D<T> ytri(ytr[i].getData(), 1, t, 1);
                        varCols += (ytri - meanCols) * (ytri - meanCols); // NOTE: Temporary assignment
                    }
                    varCols /= n_pretr;     // Compute variance
                    if (verbose) cout << "Variance of the output columns: " << endl << varCols << endl;

                    // Initialize model
                    cout << "Batch pretraining the RLS model with " << n_pretr << " samples." << endl;
                    estimator.train(Xtr, ytr);
                }
                
                catch (gException& e)
                {
                    cout << e.getMessage() << endl;
                    return false;   // Terminate program. NOTE: May be worth to set up specific error return values
                }
            }
            
            // Print detailed pretraining information
            if (verbose) 
                estimator.getOpt().printAll();
        }
        
        return true;
    }

    /************************************************************************/
    bool close()
    {        
        // Close ports
        inVec.close();
        printf("inVec closed\n");
        
        pred.close();
        printf("pred closed\n");
        
        perf.close();
        printf("perf closed\n");
        
        rpcPort.close();
        printf("rpcPort closed\n");

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
        ++updateCount;
        
        if (updateCount > numPred)
        {
            cout << "Specified number of predictions reached. Shutting down the module." << endl;
            return false;
        }

        // DEBUG

        if(verbose) cout << "updateModule #" << updateCount << endl;


        // Recursive update support and storage variables declaration and initialization
        gMat2D<T> Xnew(1,d);
        gMat2D<T> ynew(1,t);
        gVec<T> Xnew_v(d);
        gVec<T> ynew_v(t);
        gMat2D<T> *resptr = 0;
        
        // Wait for input feature vector
        if(verbose) cout << "Expecting input vector" << endl;
        
        Bottle *bin = inVec.read();    // blocking call
        
        if (bin != 0)
        {
            if(verbose) cout << "Got it!" << endl << bin->toString() << endl;

            //Store the received sample in gMat2D format for it to be compatible with gurls++

            for (int i = 0 ; i < bin->size() ; ++i)
            {
                if ( i < d )
                {
                    Xnew(0,i) = bin->get(i).asDouble();
                }
                else if ( (i>=d) && (i<d+t) )
                {

                    ynew(0, i - d ) = bin->get(i).asDouble();
                }
            }
    
            if(verbose) cout << "Xnew: " << endl << Xnew << endl;
            if(verbose) cout << "ynew: " << endl << ynew.rows() << " x " << ynew.cols() << endl;
            if(verbose) cout<< ynew << endl;

            //-----------------------------------
            //          Prediction
            //-----------------------------------
            
            // Test on the incoming sample
            resptr = estimator.eval(Xnew);
            
            Bottle& bpred = pred.prepare(); // Get a place to store things.
            bpred.clear();  // clear is important - b might be a reused object

            for (int i = 0 ; i < t ; ++i)
            {
                bpred.addDouble((*resptr)(0 , i));
            }
            
            if(verbose) printf("Sending prediction!!! %s\n", bpred.toString().c_str());
            pred.write();
            if(verbose) printf("Prediction written to port\n");

            //----------------------------------
            // performance

            Bottle& bperf = perf.prepare(); // Get a place to store things.
            bperf.clear();  // clear is important - b might be a reused object
    
            if (perfType == "nMSE")     // WARNING: The estimated variance could be unreliable...
            {
                // Compute nMSE and store
                //NOTE: In GURLS, "/" operator works like matlab's "\".
                error += varCols / ( ynew - *resptr )*( ynew - *resptr ) ;   
                gMat2D<T> tmp = error  / (updateCount);   // WARNING: Check
                for (int i = 0 ; i < t ; ++i)
                {
                    bperf.addDouble(tmp(0 , i));
                }
            }
            else if (perfType == "RMSE")
            {
                gMat2D<T> tmp(1,t);
                tmp = ( ynew - *resptr )*( ynew - *resptr );
                
                //error = ( error * (updateCount-1) + sqrt(( ynew - *resptr )*( ynew - *resptr )) ) / updateCount;
                
                error = error * (updateCount-1);
                for (int i = 0 ; i < ynew.cols() ; ++i)
                    error(0,i) += sqrt(tmp(0,i));
                error = error / updateCount;
                
/*                for (int i = 0 ; i < t ; ++i)
                {
                    bperf.addDouble(sqrt(MSE(0 , i)));
                }      */

                // WARNING: Temporary avg RMSE computation
                
                bperf.addDouble( (error(0 , 0) + error(0 , 1) + error(0 , 2))/ 3.0);    // Average MSE on forces
                bperf.addDouble( (error(0 , 3) + error(0 , 4) + error(0 , 5))/ 3.0);    // Average MSE on torques
            }
            else if (perfType == "MSE")
            {
                //Compute MSE and store
                
                error = ( error * (updateCount-1) + ( ynew - *resptr )*( ynew - *resptr ) ) / updateCount;
                for (int i = 0 ; i < t ; ++i)
                {
                    bperf.addDouble(error(0 , i));
                }
                
            }
            
            // Error storage matrix management
            // Update error storage matrix
            if (updateCount <= savedPerfNum)
            {
                gVec<T> errRow = error[0];
                storedError.setRow( errRow, updateCount-1);
            }
            
            // Save to CSV file
            if (updateCount == savedPerfNum)    
            {
                
                std::ostringstream ss;
                ss << experimentCount;
                
                //string tmp(std::to_string(experimentCount));
                storedError.saveCSV("storedError" + ss.rdbuf()->str() + ".csv");
                cout << "Error measurement matrix saved." << endl;
            }
            
            // Write computed error to output port
            if(verbose) printf("Sending %s measurement: %s\n", perfType.c_str(), bperf.toString().c_str());
            perf.write();
            
            //-----------------------------------
            //             Update
            //-----------------------------------
                        
            // Update estimator with a new input pair
            //if(verbose) std::cout << "Update # " << i+1 << std::endl;
            if(verbose) cout << "Now performing RRLS update" << endl;            
            if(verbose) cout << "Xnew" << Xnew << endl;            
            if(verbose) cout << "ynew" << ynew << endl;            
            estimator.update(Xnew, ynew);
            if(verbose) cout << "Update completed" << endl;            
        }

        if ( numPred >=0 && (updateCount == numPred) )
        {
            cout << "Specified number of predictions reached. Shutting down the module." << endl;
            return false;
        }
        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        inVec.interrupt();
        printf("inVec interrupted\n");

        pred.interrupt();
        printf("pred interrupted\n");
        
        perf.interrupt();
        printf("perf interrupted\n");
        
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
    rf.setDefaultConfigFile("RRLSestimator_config.ini");
    rf.setDefaultContext("iRRLS");
    rf.setDefault("name","RRLSestimator");
    rf.configure(argc,argv);

    // Set number of experiments
    int numPred = rf.check("numPred",Value("-1")).asInt();    
    int numExperiments = rf.check("numExperiments",Value("1")).asInt();
    if (numPred == -1)
        numExperiments = 1;
    
    // Run the module 'numExperiments' times
    for (int experimentCount = 1 ; experimentCount <= numExperiments ; ++experimentCount )
    {
        ResourceFinder rf_tmp(rf);

        RRLSestimator mod;
        rf_tmp.setDefault("experimentCount" , experimentCount);
        cout << "Experiment " << experimentCount << " started." << endl;
        int check = mod.runModule(rf_tmp);
        
        if (check != 0)
        {
            cout << "Experiment " << experimentCount << " aborted. Shutting down." << endl;
            break;
        }
        cout << "Experiment " << experimentCount << " completed successfully." << endl;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------------
