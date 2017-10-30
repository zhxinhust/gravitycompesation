#ifndef _MYMATH_
#define _MYMATH_

#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>

using namespace Eigen;

class Interval
{
public:
    double up;
    double lower;

    Interval();
    Interval(double a, double b);
    ~Interval();
};

double getrandom_interval(Interval interval);
double getrandom(double a, double b);
double absv(double x);
double get_sign(double a);
Matrix3d hatm(Vector3d a);

bool isInRange(double x, double a, double b);

Matrix4d POE(Vector4d w, Vector4d p, double theta);
//Matrix4d POE(Vector3d w, Vector3d p, double theta);


#endif
