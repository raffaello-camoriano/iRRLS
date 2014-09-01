
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

#include "gurls++/recrlswrapperchol.h"
#include "gurls++/rlsprimal.h"
#include "gurls++/primal.h"

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
    Bottle maxes;               // Max limits
    Bottle mins;                // Min limits
    int pretrain;               // Preliminary batch training required
    string pretrainFile;        // Preliminary batch training file
    int n_pretr;                // Number of pretraining samples
       
    gMat2D<T> trainSet;    
    gMat2D<T> Xtr;    
    gMat2D<T> ytr;    
    RecursiveRLSCholUpdateWrapper<T> estimator;
    gMat2D<T> varCols;          // Matrix containing the column-wise variances computed on the training set


public:
    /************************************************************************/
    RRLSestimator()
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
        perf = rf.check("perf",Value("RMSE")).asString();
        
        if ( perf != "RMSE" )
        {
            printf("Error: Inconsistent performance measure! Set to RMSE.\n");
            perf = "RMSE";
        }
        
        // Set preliminary batch training preferences
        pretrain = rf.check("pretrain",Value("0")).asInt();
        
        if ( pretrain == 1 )
        {            
            // Set preliminary batch training file path
            pretrainFile = rf.check("pretrain",Value("icubdyn.dat")).asString();
            n_pretr = rf.check("n_pretr",Value("2")).asInt();
        }
        
        // Print Configuration
        cout << endl << "-------------------------" << endl;
        cout << "Configuration parameters:" << endl << endl;
        cout << "d = " << d << endl;
        cout << "t = " << t << endl;
        cout << "perf = " << perf << endl;
        if ( pretrain == 1 )
        {
            printf("Pretraining requested\n");
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
        srand(static_cast<unsigned int>(time(NULL)));

        //Pre-training from file?
        if ( pretrain == 1 )
        {
            string trainFilePath = "data/" + pretrainFile;
            
            estimator("recursiveRLSChol");
            
            try
            {
                // Load data files
                cout << "Loading data files..." << endl;
                trainSet.readCSV(trainFilePath);
                Xtr.submatrix(trainSet , n_pretr , d);
                for ( int i = 0 ; i < t ; ++i )
                {
                    ytr.setColumn(trainSet(d + i) , i);
                }

                // Compute variance for each output on the training set
                varCols(gMat2D<T>::zeros(1,t));
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
            std::cout << e.getMessage() << std::endl;
            return EXIT_FAILURE;
        }
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
            for (int i = 0 ; i < bin->size() ; ++i)
            {
                //cout << bin->get(i).asDouble() << endl << mins.get(i).asDouble() << endl << maxes.get(i+1).asDouble() << endl;
                if (bin->get(i).asDouble() < mins.get(i).asDouble())
                    bout.add(0.0);
                else if (bin->get(i).asDouble() > maxes.get(i).asDouble())
                    bout.add(1.0);
                else
                    bout.add( ( bin->get(i).asDouble() - mins.get(i).asDouble() ) / (maxes.get(i).asDouble() - mins.get(i).asDouble() ) );
            }
            //printf("Sending %s\n", bout.toString().c_str());
            outFeatures.write();
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

    RRLSestimator mod;
    return mod.runModule(rf);
}

//-------------------------------------------------------------------------------------------------------------------------

/**
  * Main function
  *
  * The data is already split into training and test/recursive update set, and each set is
  * in the form of an input data matrix and a output labels vector.
  * Parameter selection and initial RLS estimation is carried out on a first subset of the training set.
  * Iterative RLS updates are performed on the test set after out-of-sample predictions, simulating online learning.
  * 
  */
int main(int argc, char* argv[])
{
    srand(static_cast<unsigned int>(time(NULL)));

//    std::cout.precision(16);
//    std::cout.setf( std::ios::fixed, std::ios::floatfield);
//    std::cout.setf (std::cout.showpos);

    if(argc < 2 || argc > 3)
    {
        std::cout << "Usage: " << argv[0] << " <gurls++ data directory>" << std::endl;
        return EXIT_SUCCESS;
    }

    gMat2D<T> Xtr, Xte, ytr, yte;

    std::string XtrFileName = std::string(argv[1]) + "/Xtr_id.txt";
    std::string XteFileName = std::string(argv[1]) + "/Xte_id.txt";
    std::string ytrFileName = std::string(argv[1]) + "/ytr_id.txt";
    std::string yteFileName = std::string(argv[1]) + "/yte_id.txt";

    // Set verbosity
    bool verbose = 0;
    if ( argc == 3 ) 
    {
        if (std::string(argv[2]) == "--verbose")
            verbose = 1;
    }
    
    RecursiveRLSCholUpdateWrapper<T> estimator("recursiveRLSChol");

    try
    {
        // Load data files
        std::cout << "Loading data files..." << std::endl;
        Xtr.readCSV(XtrFileName);
        Xte.readCSV(XteFileName);
        ytr.readCSV(ytrFileName);
        yte.readCSV(yteFileName);
        
        // Get data dimensions
        const unsigned long ntr = Xtr.rows();   // Number of training samples
        const unsigned long nte = Xte.rows();   // Number of test samples
        const unsigned long d = Xtr.cols();     // Number of features (dimensionality)
        const unsigned long t = ytr.cols();     // Number of outputs

        // Compute output variance for each output on the test set
        // outVar = var(yte);
        gMat2D<T> varCols(gMat2D<T>::zeros(1,t));          // Matrix containing the column-wise variances
        gVec<T>* sumCols_v = yte.sum(COLUMNWISE);          // Vector containing the column-wise sum
        gMat2D<T> meanCols(sumCols_v->getData(), 1, t, 1); // Matrix containing the column-wise sum
        meanCols /= nte;        // Matrix containing the column-wise mean
        
        if (verbose) std::cout << "Mean of the output columns: " << std::endl << meanCols << std::endl;
        
        for (int i = 0; i < nte; i++)
        {
            gMat2D<T> ytei(yte[i].getData(), 1, t, 1);
            varCols += (ytei - meanCols) * (ytei - meanCols); // NOTE: Temporary assignment
        }
        varCols /= nte;     // Compute variance
        if (verbose) std::cout << "Variance of the output columns: " << std::endl << varCols << std::endl;

        // Initialize model
        std::cout << "Batch training the RLS model with " << ntr << " samples." <<std::endl;
        estimator.train(Xtr, ytr);

        // Out-of-sample test and update RLS estimator recursively
        std::cout << "Testing and incrementally updating the RLS model with " << nte << " samples."  << std::endl;

        // Recursive update support and storage variables declaration and initialization
        gMat2D<T> Xnew(1,d);
        gMat2D<T> ynew(1,t);
        gVec<T> Xnew_v(d);
        gVec<T> ynew_v(t);
        gMat2D<T> yte_pred(nte,t);
        gMat2D<T> *resptr = 0;
        gMat2D<T> nSE(gMat2D<T>::zeros(1, t));
        gMat2D<T> nMSE_rec(gMat2D<T>::zeros(nte, t));
   
        for(unsigned long i=0; i<nte; ++i)
        {
            //-----------------------------------
            //          Prediction
            //-----------------------------------
	  
            // Read a row from the file where the test set is stored and update estimator 
            getRow(Xte.getData(), nte, d, i, Xnew.getData());
            getRow(yte.getData(), nte, t, i, ynew.getData());
	    
            // Test on the incoming sample
            resptr = estimator.eval(Xnew);
            
            // Store result in matrix yte_pred
            copy(yte_pred.getData() + i , resptr->getData(), t , nte, 1 );
            
            // Compute nMSE and store
            //WARNING: "/" operator works like matlab's "\".
            nSE += varCols / ( ynew - *resptr )*( ynew - *resptr ) ;   

            gMat2D<T> tmp = nSE  / (i+1);
            copy(nMSE_rec.getData() + i, tmp.getData(), t, nte, 1);
        
            //-----------------------------------
            //             Update
            //-----------------------------------
                        
            // Copy update sample into Xnew, ynew
            getRow(Xte.getData(), nte, d, i, Xnew.getData());
            getRow(yte.getData(), nte, t, i, ynew.getData());
                
            // Update estimator with a new input pair
            if(verbose) std::cout << "Update # " << i+1 << std::endl;
            estimator.update(Xnew, ynew);
        }
        
        // Compute average nMSE between outputs
        gVec<T>* avg_nMSE_rec_v = nMSE_rec.sum(ROWWISE);
        *avg_nMSE_rec_v /= t;
        gMat2D<T> avg_nMSE_rec(avg_nMSE_rec_v->getData() , nte , 1 , 1 );

        // Save output matrices
        std::cout << "Saving predictions matrix..." << std::endl;
        yte_pred.saveCSV("yte_pred.txt");
        
        std::cout << "Saving performance matrices..." << std::endl;
        avg_nMSE_rec.saveCSV("avg_nMSE_rec.txt");
        nMSE_rec.saveCSV("nMSE_rec.txt");

        std::cout << "Done! Now closing." << std::endl;
        return EXIT_SUCCESS;
    }
    catch (gException& e)
    {
        std::cout << e.getMessage() << std::endl;
        return EXIT_FAILURE;
    }
}