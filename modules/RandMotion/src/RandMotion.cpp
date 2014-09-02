
void AffManager::handToCenter()
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