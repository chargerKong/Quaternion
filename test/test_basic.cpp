#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif
//#include <opencv2/opencv.hpp>
//using namespace cv;

void test_operator()
{
	
	Quaternion<double> q1(cv::Vec<double, 4>{0,2,3,4});
	Quaternion<double> q2(cv::Vec<double, 4>{0,2,3,4});
	double scalar = 3.1;
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
	assert(q1.getCoeff() + q2.getCoeff() == (q1 + q2).getCoeff());
	assert(q1.getCoeff() - q2.getCoeff() == (q1 - q2).getCoeff());
	assert(q1.getCoeff() / scalar == (q1 /scalar).getCoeff());
	assert(q1.getCoeff() * q2.getCoeff() == (q1 * q2).getCoeff());
	assert(q2.getCoeff() * q1.getCoeff() == (q2 * q1).getCoeff());
	assert(q1 * scalar == scalar * q1);
	const Quaternion<double> q3 = q1;
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
	
	// std::cout << "q1.getRotMat(30,[1,2,3]) = " <<
	// UnitQuaternion<double>::getRotMat(30, cv::Vec<double, 3>{1,2,3} /
	// sqrt(14))<< std::endl;
	
	// q1.transform(30, cv::Vec<double, 3>{1,2,3} / sqrt(14));
	// std::cout << "after transformation" << q1 << std::endl;
	
	Quaternion<double> q4{0.4,1,2,3};
	std::cout << q4 * q1 * q4.conjugate()<< std::endl;

}
