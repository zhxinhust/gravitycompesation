//
// Created by zhaoxin on 17-10-25.
//

#ifndef ATI_MYATI_H
#define ATI_MYATI_H

#include "pmd.h"
#include "usb-1608FS-Plus.h"
#include "ftconfig.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <kdl/frames.hpp>

using namespace Eigen;
using namespace geometry_msgs;
using namespace KDL;

class ATIGravityCompensation{
public:
    void calParam(MatrixXd);
    KDL::Wrench compensete(KDL::Wrench F, Rotation R);
    void initCalParam();

    Vector3d F0;
    Vector3d T0;
    Vector3d L;
    double G;
};

class myATI
{
public:
    int initATI();
    void forcemeasure(float *FT);
    void biasmeasure();
    void readBias(float *base);


    float table_AIN[NGAINS_USB1608FS_PLUS][NCHAN_USB1608FS_PLUS][2];
    float forcebias[6];
    float forcebiastemp[6];

    ATIGravityCompensation gravityCompen;
    libusb_device_handle *udev;
    Calibration *cal;		  // struct containing calibration information
};



#endif //ATI_MYATI_H
