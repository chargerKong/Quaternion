#include <iostream>
#include <opencv2/core/quaternion.hpp>
using namespace cv;
int main()
{
    Mat point = (Mat_<double> (1, 3) << 1,0,0);
    point = point.t();
    Quatd q0 = Quatd::createFromAngleAxis(0, Vec3d{0,0,1});
    Quatd q1 = Quatd::createFromAngleAxis(CV_PI * 2/ 3, Vec3d{1,1,1});
    Quatd q2 = Quatd::createFromAngleAxis(CV_PI, Vec3d{0,0,1});
    Quatd q3 = Quatd::createFromAngleAxis(CV_PI / 2, Vec3d{1,-2, 3});
    Quatd q4{1,2,3,4};
    for (double i = 0; i < 21; ++i)
    {
        //Quatd qt = Quatd::slerp(q0, q1, i / 10);
        Quatd qt = Quatd::spline(q0, q0, q1, q2, i / 20);
        Mat points = qt.toRotMat3x3() * point;
        std::cout << points.t() << std::endl;
    }

    
    for (double i = 0; i < 21; ++i)
    {
        //Quatd qt = Quatd::slerp(q1, q2, i / 10);
        Quatd qt = Quatd::spline(q0, q1, q2, q3, i / 20);
        Mat points = qt.toRotMat3x3() * point;
        std::cout << points.t() << std::endl;
    }

    
    for (double i = 0; i < 21; ++i)
    {
        //Quatd qt = Quatd::slerp(q2, q3, i / 10);
        Quatd qt = Quatd::spline(q1, q2, q3, q3, i / 20);
        Mat points = qt.toRotMat3x3() * point;
        std::cout << points.t() << std::endl;
    }
    return 0;
}

