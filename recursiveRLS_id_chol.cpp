/*
 * The GURLS Package in C++
 *
 * Copyright (C) 2011-1013, IIT@MIT Lab
 * All rights reserved.
 *
 * authors:  Raffaello Camoriano
 * email:   raffaello.camoriano@iit.it
 * website: http://cbcl.mit.edu/IIT@MIT/IIT@MIT.html
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name(s) of the copyright holders nor the names
 *       of its contributors or of the Massacusetts Institute of
 *       Technology or of the Italian Institute of Technology may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \ingroup Tutorials
 * \file
 */

#include <iostream>
#include <string>

#include "gurls++/recrlswrapperchol.h"
#include "gurls++/rlsprimal.h"
#include "gurls++/primal.h"

using namespace gurls;
typedef double T;

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