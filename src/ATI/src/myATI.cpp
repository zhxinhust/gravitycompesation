//
// Created by zhaoxin on 17-10-25.
//
#include <iostream>
#include "myATI.h"


int myATI::initATI()
{
    //char *calfilepath;      // name of calibration file
    char *calfilepath = "/home/zhaoxin/Temp/ATI_force_read/src/ATI/src/calibrationfiles/FT20582.cal";      // name of calibration file

    unsigned short index = 1;   // index of calibration in file (second parameter; default = 1)
    short sts;              // return value from functions
    int ret;
    struct tm calDate;
    int  j;
    int i;
    float SampleTT[6]={0,0,0,0,0,0};
    FILE *biasfile;

    libusb_device_handle *udev2;

    i = 0;

    // 读取力初始值，如果读取文件失败，则将所有值置为零
    biasfile = fopen("./src/calibrationfiles/forcebias.txt", "r");
    if(biasfile == NULL)
    {
        forcebias[0] = 0;
        forcebias[1] = 0;
        forcebias[2] = 0;
        forcebias[3] = 0;
        forcebias[4] = 0;
        forcebias[5] = 0;
    }
    else
    {
        fscanf(biasfile, "%f, %f, %f, %f, %f, %f", &forcebias[0], &forcebias[1]
                , &forcebias[2], &forcebias[3]
                , &forcebias[4], &forcebias[5]);
        fclose(biasfile);
    }

    forcebiastemp[0] = 0;
    forcebiastemp[1] = 0;
    forcebiastemp[2] = 0;
    forcebiastemp[3] = 0;
    forcebiastemp[4] = 0;
    forcebiastemp[5] = 0;

    start:
    udev = NULL;
    udev2 = NULL;

    ret = libusb_init(NULL);
    if (ret < 0)
    {
        perror("usb_device_find_USB_MCC: Failed to initialize libusb");
        exit(1);
    }

    if ((udev = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL)))
    {
        printf("Success, found a USB 1608FS-Plus!\n");
    }
    else
    {
        printf("Failure, did not find a USB 1608FS-Plus!\n");
        return 0;
    }
    /******************************** Finding a second device has issues on the Raspberry Pi **************/
    // See if there is a second device:
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000103)
    if ((udev2 = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL)))
	{
		printf("Success, found a second USB 1608FS-Plus!\n");
	}
	else
	{
		printf("Did not find a second device.\n");
	}
#endif

    // some initialization
    //print out the wMaxPacketSize.  Should be 64.
    printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

    usbBuildGainTable_USB1608FS_Plus(udev, table_AIN);
    for (i = 0; i < NGAINS_USB1608FS_PLUS; i++ )
    {
        for (j = 0; j < NCHAN_USB1608FS_PLUS; j++)
        {
            printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n",
                   i, j, table_AIN[i][j][0], table_AIN[i][j][1]);
        }
    }
    usbCalDate_USB1608FS_Plus(udev, &calDate);
    printf("\n");

    // create Calibration struct
    cal = createCalibration(calfilepath, index);
    if (cal==NULL)
    {
        printf("\nSpecified calibration could not be loaded.\n");
        scanf(".");
        return 0;
    }
    // Set force units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts = SetForceUnits(cal, "N");
    switch (sts)
    {
        case 0: break;	// successful completion
        case 1: printf("Invalid Calibration struct");
            return 0;
        case 2: printf("Invalid force units");
            return 0;
        default: printf("Unknown error");
            return 0;
    }
    // Set torque units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts = SetTorqueUnits(cal, "N-m");
    switch (sts)
    {
        case 0: break;	// successful completion
        case 1: printf("Invalid Calibration struct");
            return 0;
        case 2: printf("Invalid torque units");
            return 0;
        default: printf("Unknown error");
            return 0;
    }

    // Set tool transform.
    // This line is only required if you want to move or rotate the sensor's coordinate system.
    // This example tool transform translates the coordinate system 20 mm along the Z-axis
    // and rotates it 45 degrees about the X-axis.
    sts = SetToolTransform(cal,SampleTT,"mm","degrees");
    switch (sts)
    {
        case 0: break;	// successful completion
        case 1: printf("Invalid Calibration struct"); return 0;
        case 2: printf("Invalid distance units"); return 0;
        case 3: printf("Invalid angle units"); return 0;
        default: printf("Unknown error");
            return 0;
    }

    MatrixXd gravityParam;
    gravityParam.resize(6, 6);
    gravityParam <<  -0.00113924,   0.00139937,   0.00831386,  1.81385e-05, -2.38852e-05, -0.000351935,
                     -0.00972399,  -0.192853,     12.029,  0.0388635,  0.0739638, 0.00105526,
                     0.0459536,  -6.06178,   6.01534, 0.0934705, 0.0377947, 0.0378077,
                     5.95626,  -0.148628,    5.96515,  0.0263904,   0.109853, -0.0167116,
                     0.0465602,  -6.04175,   5.99649, 0.0942382, 0.0377341, 0.0370656,
                     0.140435,    5.90709,    5.94931, -0.0529019,  0.0381441, -0.0402472;

    gravityCompen.calParam(gravityParam);
}

void myATI::readBias(float *base)
{
    int i;
    int j;
    int k;

    int nchan;
    int frequency = 1000;	// 采样频率
    int ret;
    uint32_t count = 1;
    uint16_t sdataIn[8*512]; // holds 16 bit unsigned analog input data
    uint16_t data;
    uint8_t range;
    uint8_t ranges[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t channels;
    float reading[6];

    float F[6];

    nchan = 6;		//  6个通道
    channels = 0;

    for(i = 0; i < nchan; i++)
    {
        channels |= (1 << i);
    }

    memset(ranges, 0, sizeof(ranges));
    usbAInScanConfig_USB1608FS_Plus(udev, ranges);

    usbAInScanStart_USB1608FS_Plus(udev, count, frequency, channels, 0);

    ret = usbAInScanRead_USB1608FS_Plus(udev, count, nchan, sdataIn, 0);

    if (ret != count * nchan * 2)
    {
        printf("***ERROR***  ret = %d   count = %d  nchan = %d\n", ret, count, nchan);
    } /* if (ret != count * nchan * 2) */

    for (i = 0; i < count; i++)
    {
        for (j = 0; j < nchan; j++)
        {
            k = i * nchan + j;

            data = rint(sdataIn[k] * table_AIN[i][j][0] + table_AIN[i][j][1]);
            *(base + j) = volts_USB1608FS_Plus(data, i);
        } /* for (j - 0; j < 8, j++) */

    } /* for (i = 0; i < count; i++) */

}

void myATI::biasmeasure()
{
    float baseTemp[6];
    int readTimes = 5000;

    // 先将bias里面的值清零
    for(int i = 0; i < 6; i++)
        forcebias[i] = 0;

    std::cout << "reading bias, please wait..." << std::endl;
    for(int i = 0; i < readTimes; i++)
    {
        readBias(baseTemp);
        for(int j = 0; j < 6; j++)
            forcebias[j] += baseTemp[j];

        usleep(1000);
    }

    for(int j = 0; j < 6; j++)
        forcebias[j] = forcebias[j] / (float) readTimes;

    std::cout << "bias in readed!" << std::endl;

    Bias(cal, forcebias);
}

void myATI::forcemeasure(float *FT)
{
    float reading[6];
    readBias(reading);
    ConvertToFT(cal, reading, FT);
}

void ATIGravityCompensation::calParam(MatrixXd M)
{
    // 计算初始重力
    F0[0] = (M(2, 0) + M(3, 0)) / 2.0;
    F0[1] = (M(4, 1) + M(5, 1)) / 2.0;
    F0[2] = (M(0, 2) + M(1, 2)) / 2.0;

    // 计算力矩
    T0[0] = (M(0, 3) + M(1, 3) + M(4, 3) + M(5, 3)) / 4.0;
    T0[1] = (M(0, 4) + M(1, 4) + M(2, 4) + M(3, 4)) / 4.0;
    T0[2] = (M(2, 5) + M(3, 5) + M(4, 5) + M(5, 5)) / 4.0;

    G = (M(1, 2) - M(0, 2) + M(3, 0) - M(2, 0) + M(5, 1) - M(4, 1)) / 6.0;

    L[0] = (M(0, 4) - M(1, 4) + M(5, 5) - M(4, 5)) / 4.0 / G;
    L[1] = (M(1, 3) - M(0, 3) + M(2, 5) - M(3, 5)) / 4.0 / G;
    L[2] = (M(3, 4) - M(2, 4) + M(4, 3) - M(5, 3)) / 4.0 / G;
}

KDL::Wrench ATIGravityCompensation::compensete(KDL::Wrench F, Rotation R)
{
//    Twist twist;
    KDL::Wrench w;
    Vector G_v = G * Vector(R.data[6], R.data[7], R.data[8]);

    w.force.data[0] = F.force.x() + G_v[0] - F0[0];
    w.force.data[1] = F.force.y() + G_v[1] - F0[1];
    w.force.data[2] = F.force.z() + G_v[2] - F0[2];

    w.torque[0] = F.torque[0] + G_v[2] * L[1] - G_v[1] * L[2] - T0[0];
    w.torque[1] = F.torque[1] + G_v[0] * L[2] - G_v[2] * L[0] - T0[1];
    w.torque[2] = F.torque[2] + G_v[1] * L[0] - G_v[0] * L[1] - T0[2];

    return w;
}

void ATIGravityCompensation::initCalParam()
{
    MatrixXd gravityParamMat;
    gravityParamMat.resize(6, 6);
    gravityParamMat << -0.0143091, 0.00162719, -0.0376344, 0.000150722, 0.000151516, 0.000124774,
    -0.0129553, -0.186113, 12.0042, 0.0398065, 0.0741769, 0.0024371,
    -5.9949, -0.227455, 5.95385, 0.0177498, -0.0376208, 0.0193341,
    5.95129, -0.154683, 5.87497, 0.0268499, 0.110702, -0.0146823,
    0.0529635, -6.05799, 5.877, 0.0954623, 0.0375333, 0.0398577,
    0.137182, 5.9378, 5.79055, -0.0522352, 0.0378923, -0.0372377;
    calParam(gravityParamMat);
}