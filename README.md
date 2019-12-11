iRRLS
==========
Recursive Regularized Least Squares for iCub
----------
Author:
- Raffaello Camoriano

This package is constituted by a recursive regularized least squares (RRLS) estimator based on the GURLS++ machine learning software framework. The estimator can be optionally pre-trained in batch mode using a known training set. Furthermore, it can be incrementally updated with new incoming samples in order to refine its predictions and avoid model deterioration in relatively long time spans.

The application includes a demonstration of the inverse dynamics learning for the iCub humanoid arm.

Tutorial
----------

[![Video Tutorial](https://img.youtube.com/vi/ca8NW16c4G8/0.jpg)](https://www.youtube.com/watch?v=ca8NW16c4G8)

How to run the Random Motion module on the iCub simulator

1) Run "yarpserver"
2) Run "iCub_SIM"
3) Run "simCartesianControl --robot icubSim"
4) Run "iKinCartesianSolver --context simCartesianControl --part left_arm"
5) Run "RandMotion --from RandMotion_config_SIM.ini"
