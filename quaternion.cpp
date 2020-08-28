#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif

#include <assert.h>
#include <iostream>
#include <math.h>
#include "test/test_basic.cpp"
//using namespace cv;

Quaternion::Quaternion(cv::Vec4d coeff):coeff(coeff){}

Quaternion::Quaternion(double qw, double qx, double qy, double qz):coeff(qw, qx, qy, qz){}

Quaternion::Quaternion(cv::Mat &R)
{
	assert(R.rows == 3 && R.cols == 3);
	double qw, qx, qy, qz, S;
	double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
	if (trace > 0.00000001f)
	{
		S = sqrt(trace) * 2; // TBD
		qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) / S;
		qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) / S;
		qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) / S;
		qw = 0.25 * S;
	}
	else
	{
		if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2))
		{
			S = sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2)) * 2;
			qx = 0.25 * S;
			qy = (R.at<double>(1, 0) + R.at<double>(0, 1)) / S;
			qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / S;
			qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / S;
		}
		else if (R.at<double>(1, 1) > R.at<double>(2, 2))
		{
			S = sqrt(1.0 - R.at<double>(0, 0) + R.at<double>(1, 1) - R.at<double>(2, 2)) * 2;
			qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / S; 
			qy = 0.25 * S;
			qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / S;
			qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / S;
		}
		else
		{
			S = sqrt(1.0 - R.at<double>(0, 0) - R.at<double>(1, 1) + R.at<double>(2, 2)) * 2;
			qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / S;
			qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / S;
			qz = 0.25 * S;
			qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / S;
		}
	}
	coeff = {qw, qx, qy, qz}; 
}

inline bool Quaternion::operator==(const Quaternion &q) const
{
	return coeff == q.coeff;
}

inline Quaternion Quaternion::operator+(const Quaternion &q1) const
{
	return Quaternion(coeff + q1.coeff);
}

inline Quaternion Quaternion::operator-(const Quaternion &q1) const
{
	return Quaternion(coeff - q1.coeff);
}

inline Quaternion& Quaternion::operator+=(const Quaternion &q1)
{
	coeff += q1.coeff;
	return *this;
}

inline Quaternion& Quaternion::operator-=(const Quaternion &q1)
{
	coeff -= q1.coeff;
	return *this;
}


inline Quaternion Quaternion::operator*(const Quaternion &q1) const
{
	return Quaternion(coeff * q1.coeff);
}

Quaternion operator*(const Quaternion &q1, const double a)
{
	return Quaternion(q1.coeff * a);
}

Quaternion operator*(const double a, const Quaternion &q1)
{
	return q1 * a;
}

inline Quaternion& Quaternion::operator*=(const Quaternion &q1)
{
	coeff *= q1.coeff;
	return *this;
}

inline Quaternion& Quaternion::operator*=(const double &q1)
{
	coeff *= q1;
	return *this;
}


inline Quaternion& Quaternion::operator/=(const double &a)
{
	coeff /= a;
	return *this;
}


inline Quaternion Quaternion::operator/(const double &a) const
{
	return Quaternion(coeff / a);
}


inline const double& Quaternion::operator[](std::size_t n) const
{
	return coeff[n];
}

inline double& Quaternion::operator[](std::size_t n)
{
	return coeff[n];
}


std::ostream &operator<<(std::ostream &os, const Quaternion &q)
{
	os << q.coeff;
	return os;
}

inline Quaternion Quaternion::conjugate() const
{
	return Quaternion(coeff[0], -coeff[1], -coeff[2], -coeff[3]);
}

inline double Quaternion::norm() const
{
	return sqrt(coeff.dot(coeff));
}

inline double Quaternion::dot(Quaternion &q) const
{
	return coeff.dot(q.coeff);
} 

inline Quaternion Quaternion::normalize() const
{
	return Quaternion(coeff / norm());
}

inline Quaternion Quaternion::inv() const
{
	return this->conjugate() / pow(this->norm(), 2);
}



int main(){
	cv::Mat R(3, 3, 5);
	Quaternion q1(R);
	test_operator();
	return 0;
}
