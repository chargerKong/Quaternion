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

Quaternion::Quaternion(double v1, double v2, double v3, double v4):coeff({v1, v2, v3, v4}){}

inline
bool Quaternion::operator==(const Quaternion &q) const
{
	return coeff == q.coeff;
}

inline
Quaternion Quaternion::operator+(const Quaternion &q1) const
{
	return Quaternion(coeff + q1.coeff);
}

inline
Quaternion Quaternion::operator-(const Quaternion &q1) const
{
	return Quaternion(coeff - q1.coeff);
}

inline
Quaternion& Quaternion::operator+=(const Quaternion &q1)
{
	coeff += q1.coeff;
	return *this;
}

inline
Quaternion& Quaternion::operator-=(const Quaternion &q1)
{
	coeff -= q1.coeff;
	return *this;
}


inline
Quaternion Quaternion::operator*(const Quaternion &q1) const
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

inline
Quaternion& Quaternion::operator*=(const Quaternion &q1)
{
	coeff *= q1.coeff;
	return *this;
}

inline
Quaternion& Quaternion::operator*=(const double &q1)
{
	coeff *= q1;
	return *this;
}


inline
Quaternion& Quaternion::operator/=(const double &a)
{
	coeff /= a;
	return *this;
}


inline
Quaternion Quaternion::operator/(const double &a) const
{
	return Quaternion(coeff / a);
}


inline
const double& Quaternion::operator[](std::size_t n) const
{
	return coeff[n];
}

inline
double& Quaternion::operator[](std::size_t n)
{
	return coeff[n];
}


std::ostream &operator<<(std::ostream &os, const Quaternion &q)
{
	os << q.coeff;
	return os;
}

inline
Quaternion Quaternion::conjugate() const
{
	return Quaternion(coeff[0], -coeff[1], -coeff[2], -coeff[3]);
}

inline 
double Quaternion::norm() const
{
	return sqrt(coeff.dot(coeff));
}

inline
double Quaternion::dot(Quaternion &q) const
{
	return coeff.dot(q.coeff);
} 

inline
Quaternion Quaternion::normalize() const
{
	return Quaternion(coeff / norm());
}

inline
Quaternion Quaternion::inv() const
{
	return this->conjugate() / pow(this->norm(), 2);
}



int main(){
	test_operator();
	return 0;
}
