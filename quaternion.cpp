#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif

#include <assert.h>
//#nclude <iostream>
#include <math.h>
#include "test/test_basic.cpp"
//using namespace cv;

template <typename T>
Quaternion<T>::Quaternion(cv::Vec<T, 4> coeff):coeff(coeff){}

template <typename T>
Quaternion<T>::Quaternion(T qw, T qx, T qy, T qz):coeff(qw, qx, qy, qz){}

template <typename T>
Quaternion<T>::Quaternion(cv::Mat &R):R(R){}
/*
template <typename T>
Quaternion<T>::Quaternion(cv::Mat &R)
{
	// 确定Mat是什么类型
	assert(R.rows == 3 && R.cols == 3);
	T qw, qx, qy, qz, S;
	T trace = R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2);
	if (trace > 0.00000001f)
	{
		S = sqrt(trace) * 2; // TBD
		qx = (R.at<T>(2, 1) - R.at<T>(1, 2)) / S;
		qy = (R.at<T>(0, 2) - R.at<T>(2, 0)) / S;
		qz = (R.at<T>(1, 0) - R.at<T>(0, 1)) / S;
		qw = 0.25 * S;
	}
	else
	{
		if (R.at<T>(0, 0) > R.at<T>(1, 1) && R.at<T>(0, 0) > R.at<T>(2, 2))
		{
			S = sqrt(1.0 + R.at<T>(0, 0) - R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
			qx = 0.25 * S;
			qy = (R.at<T>(1, 0) + R.at<T>(0, 1)) / S;
			qz = (R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
			qw = (R.at<T>(2, 1) - R.at<T>(1, 2)) / S;
		}
		else if (R.at<T>(1, 1) > R.at<T>(2, 2))
		{
			S = sqrt(1.0 - R.at<T>(0, 0) + R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
			qx = (R.at<T>(0, 1) + R.at<T>(1, 0)) / S; 
			qy = 0.25 * S;
			qz = (R.at<T>(1, 2) + R.at<T>(2, 1)) / S;
			qw = (R.at<T>(0, 2) - R.at<T>(2, 0)) / S;
		}
		else
		{
			S = sqrt(1.0 - R.at<T>(0, 0) - R.at<T>(1, 1) + R.at<T>(2, 2)) * 2;
			qx = (R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
			qy = (R.at<T>(1, 2) + R.at<T>(2, 1)) / S;
			qz = 0.25 * S;
			qw = (R.at<T>(1, 0) - R.at<T>(0, 1)) / S;
		}
	}
	coeff = {qw, qx, qy, qz}; 
}
*/

template <typename T>
inline bool Quaternion<T>::operator==(const Quaternion<T> &q) const
{
	return coeff == q.coeff;
}

template <typename T>
inline Quaternion<T> Quaternion<T>::operator+(const Quaternion<T> &q1) const
{
	return Quaternion<T>(coeff + q1.coeff);
}

template <typename T>
inline Quaternion<T> Quaternion<T>::operator-(const Quaternion<T> &q1) const
{
	return Quaternion<T>(coeff - q1.coeff);
}

template <typename T>
inline Quaternion<T>& Quaternion<T>::operator+=(const Quaternion<T> &q1)
{
	coeff += q1.coeff;
	return *this;
}

template <typename T>
inline Quaternion<T>& Quaternion<T>::operator-=(const Quaternion<T> &q1)
{
	coeff -= q1.coeff;
	return *this;
}


template <typename T>
inline Quaternion<T> Quaternion<T>::operator*(const Quaternion<T> &q1) const
{
	return Quaternion<T>(coeff * q1.coeff);
}

template <typename T>
Quaternion<T> operator*(const Quaternion<T> &q1, const T a)
{
	return Quaternion<T>(q1.coeff * a);
}

template <typename T>
Quaternion<T> operator*(const T a, const Quaternion<T> &q1)
{
	return q1 * a;
}

template <typename T>
inline Quaternion<T>& Quaternion<T>::operator*=(const Quaternion<T> &q1)
{
	coeff *= q1.coeff;
	return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator*=(const T &q1)
{
	coeff *= q1;
	return *this;
}

template <typename T>
inline Quaternion<T>& Quaternion<T>::operator/=(const T &a)
{
	coeff /= a;
	return *this;
}

template <typename T>
inline Quaternion<T> Quaternion<T>::operator/(const T &a) const
{
	return Quaternion<T>(coeff / a);
}

template <typename T>
inline const T& Quaternion<T>::operator[](std::size_t n) const
{
	return coeff[n];
}

template <typename T>
inline T& Quaternion<T>::operator[](std::size_t n)
{
	return coeff[n];
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const Quaternion<T> &q)
{
	os << q.coeff;
	return os;
}

template <typename T>
inline Quaternion<T> Quaternion<T>::conjugate() const
{
	return Quaternion<T>(coeff[0], -coeff[1], -coeff[2], -coeff[3]);
}

template <typename T>
inline T Quaternion<T>::norm() const
{
	return sqrt(coeff.dot(coeff));
}

template <typename T>
inline T Quaternion<T>::dot(Quaternion<T> &q) const
{
	return coeff.dot(q.coeff);
} 

template <typename T>
inline Quaternion<T> Quaternion<T>::normalize() const
{
	return Quaternion<T>(coeff / norm());
}

template <typename T>
inline Quaternion<T> Quaternion<T>::inv() const
{
	return this->conjugate() / pow(this->norm(), 2);
}

template <typename T>
cv::Mat Quaternion<T>::getRotMat(const T &angle, const cv::Vec<T, 3> &axis)
{
	assert(Quaternion<T>::isNormalized(axis) == true);
	// if (Quaternion<T>::isNormalized(axis) == false)
	// {
	//     raise("balabala")
	// }
	T a = cos(angle / 2), b = sin(angle / 2) * axis[1], c = sin(angle / 2) * axis[2], d = sin(angle / 2) * axis[3];
	cv::Matx<T, 4, 4> R{
		 1, 0,                       0,                       0,
		 0, 1 - 2 * (c * c + d * d), 2 * (b * c - a * d),      2 * (a * c + b * d),
		 0, 2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * d),
		 0, 2 * (b * d - a * c)    , 2 + (a * b + c * d),     1 - 2 * (b * b + c * c)};
	return cv::Mat(R); 
}

template <typename T>
void Quaternion<T>::transform(const T &angle, const cv::Vec<T, 3> &axis)
{
	coeff.t();
	cv::Mat rotMat = getRotMat(angle, axis) * coeff;
	cv::Vec<T, 4> abc(rotMat);
	coeff = abc;
	std::cout << coeff << std::endl;
}

template <typename T>
inline Quaternion<T> Quaternion<T>::getRotQuat(const T& angle, const cv::Vec<T, 3> &axis)
{
	assert(Quaternion<T>::isNormalized(axis) == true);
	// if (Quaternion<T>::isNormalized(axis) == false)
	// {
	//     raise("balabala")
	// }
	return Quaternion(sin(angle / 2), cos(angle / 2) * axis[0], cos(angle / 2) * axis[1], cos((angle / 2) * axis[2]));
}

template <typename T>
template <typename _Tp>
bool Quaternion<T>::isNormalized(_Tp obj)
{
	double eps = 0.0000001;
	double normVar = sqrt(obj.dot(obj));
	std::cout << "obj = " << obj << "normVar = " << normVar << std::endl;
	if ((normVar > 1 - eps) && (normVar < 1 + eps))
		return true;
	return false;
}


int main(){
	test_operator();
	std::cout << "=================================" << std::endl;
	Quaternion<float> q1{2,1,3,4};
	q1 = q1.normalize();
	std::cout << q1.norm() << std::endl;
	
	bool ans = Quaternion<double>::isNormalized(cv::Vec<double, 4>{1,2,3,4});
	assert(ans == false);
	ans = Quaternion<double>::isNormalized(Quaternion<double>{1.1,2,3,4});
	assert(ans == false);
	ans = Quaternion<double>::isNormalized(Quaternion<double>{1,2,3,4} / sqrt(30));
	assert(ans == true);
	ans = Quaternion<double>::isNormalized(cv::Vec<double, 4>{1,2,3,4} / sqrt(30));
	assert(ans == true);
	
	Quaternion<double>::getRotQuat(3.1415926/2, cv::Vec<double, 3>{1 ,2 ,3} / sqrt(14));
	std::cout << ans << std::endl;
	return 0;
}
