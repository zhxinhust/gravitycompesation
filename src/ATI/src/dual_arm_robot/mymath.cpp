#include "mymath.hpp"
#include<math.h>
#include<time.h>

double getrandom_interval(Interval interval)
{
    return getrandom(interval.lower, interval.up);
}

// 生成随机数
double getrandom(double a, double b)
{
    int randomi = rand();     // 生成随机数
    double out;
    out = a + double(randomi) / double(RAND_MAX) * (b - a);
    return out;
}

/**************************************************
 * 绝对值函数
 * ***********************************************/
double absv(double x)
{
    if(x > 0)
        return x;
    else
        return -x;
}

Matrix3d hatm(Vector3d a)
{
    Matrix3d m;
    m<< 0, -a[2], a[1],
    a[2], 0, -a[0],
    -a[1], a[0], 0;
    return m;
}

Matrix4d POE(Vector4d w4, Vector4d p4, double theta)
{
    Matrix4d POEMatrix;
    Vector3d w = w4.block(0, 0, 3, 1);
    Vector3d p = p4.block(0, 0, 3, 1);

    Matrix3d R;
    Vector3d q;
    Vector3d v;

    v = -w.cross(p);

    R = Matrix3d::Identity() + hatm(w) * sin(theta) + hatm(w) * hatm(w) * (1 - cos(theta));

    q = (Matrix3d::Identity() - R) * w.cross(v) + w * w.dot(v) * theta;

    POEMatrix.block(0, 0, 3, 3) = R;
    POEMatrix.block(0, 3, 3, 1) = q;
    POEMatrix(3, 3) = 1;
    return POEMatrix;
}

double get_sign(double a)
{
    if(a > 0)
        return 1;
    else if(a < 0)
        return -1;
    else
        return 0;
}

/**
 * 判断一个数x是否在区间[a, b]中
 * @param x
 * @param a
 * @param b
 * @return 如果在区间里，则返回true，否则返回false
 */
bool isInRange(double x, double a, double b)
{
    if(x >= a && x <= b)
        return true;
    else
        return false;
}

Interval::Interval(){}
Interval::~Interval() {}

Interval::Interval(double a, double b)
{
    up = b;
    lower = a;
}




