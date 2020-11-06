#include <iostream>
#include "quaternion.hpp"
using namespace cv;
int main()
{
    Quatd q1{1,2,3,4};
    DualQuatd dq1{1,2,3,4,5,6,7,8};
    DualQuatd dq2 = dq1.normalize();
    Quatd a = q1.inv() * q1;
    Quatd real = dq2.getRealQuat();
    Quatd dual = dq2.getDualQuat();
    std::cout << "real:" <<real << std::endl;
    std::cout << "dual:" <<dual << std::endl;
    std::cout <<"total:"<< dq2 * DualQuatd::createFromQuat(real.inv(), -real.inv()*dual*real.inv()) <<std::endl;
    std::cout <<  real *real.inv()<< std::endl;
    std::cout <<  real *real.inv() *dual *real.inv()<< std::endl;
    std::cout << dual * real.inv()<< std::endl;
    return 0;
}

