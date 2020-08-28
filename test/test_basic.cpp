#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif
//#include <opencv2/opencv.hpp>
//using namespace cv;

void test_operator()
{
	
	Quaternion q1(cv::Vec4d{1,2,3,4});
	cv::Mat R = (cv::Mat_<uchar>(3,3) << -9, 10, 0, 6, -9, 8, 8, 0, -15);
	Quaternion q2(R);
	double scalar = 3.1;
	std::cout << R.at<double>(0,2) << std::endl;
	std::cout << "q2:" << q2  << std::endl;
	std::cout << "q1:" << q1  << std::endl;
	std::cout << "scalar:" << scalar << std::endl;

	std::cout << "q1 + q2=:" << (q1 + q2)  << std::endl;
	std::cout << "q1 - q2=:" << (q1 - q2)  << std::endl;
	std::cout << "q1 * q2=:" << (q1 * q2)  << std::endl;
	std::cout << "q2 * q1=:" << (q2 * q1)  << std::endl;
	std::cout << "scalar * q2=:" << (scalar * q2)  << std::endl;
	std::cout << "q2 * scalar=:" << (q2 * scalar)  << std::endl;
	std::cout << "q1 / scalar=:" << (q1 / scalar)  << std::endl;
	assert(q1.coeff + q2.coeff == (q1 + q2).coeff);
	assert(q1.coeff - q2.coeff == (q1 - q2).coeff);
	assert(q1.coeff / scalar == (q1 /scalar).coeff);
	assert(q1.coeff * q2.coeff == (q1 * q2).coeff);
	assert(q2.coeff * q1.coeff == (q2 * q1).coeff);
	assert(q1 * scalar == scalar * q1);
	const Quaternion q3 = q1;
	q1 += q2;
	std::cout << "q1 += q2, q1 = " << q1 << std::endl;
	assert(q1 == q3 + q2);
	q1 = q3;
	q1 -= q2;
	std::cout << "q1 -= q2, q1 = " << q1 << std::endl;
	assert(q1 == q3 - q2);

	q1 = q3;
	q1 *= q2;
	std::cout << "q1 *= q2, q1 = " << q1 << std::endl;

	q1 = q3;
	q1 /= scalar;
	std::cout << "q1 /= scalar, q1 = " << q1 << std::endl;
	
	q1 = q3;
	std::cout << "q1.conjugate():"<< q1.conjugate()  << std::endl;
    
	std::cout << "q1.dot(q2) = " << q1.dot(q2) << std::endl;
	assert(q1.dot(q2) == q1 .dot(q2 ));
	std::cout << "q1.norm():" << q1.norm() << std::endl;
	
	std::cout << "q1 * q1.conjugate():" << (q1*q1.conjugate())  << std::endl;
	std::cout << "q1.conjugate() * q1:" << (q1.conjugate()*q1)  << std::endl;
	assert((q1 * q1.conjugate()) == (q1.conjugate() * q1));

	std::cout << "q1.normalze()" << q1.normalize() << std::endl;
	assert (q1.normalize() == q1 / q1.norm());

	std::cout << "q1[0]:" << q1[0] << std::endl;

	std::cout <<"q1 * q1.inv() = " <<  q1 * q1.inv() << std::endl;
	
}
