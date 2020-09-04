#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif

#include <assert.h>
//#nclude <iostream>
#include <math.h>
#include "test/test_basic.cpp"
#include "test/test_unit_basic.cpp"
// using namespace cv;

template <typename T>
Quaternion<T>::Quaternion(cv::Vec<T, 4> coeff):coeff(coeff){}

template <typename T>
Quaternion<T>::Quaternion(T qw, T qx, T qy, T qz):coeff(qw, qx, qy, qz){}

template <typename T>
Quaternion<T>::Quaternion(const T angle, const cv::Vec<T, 3> &axis)
{
	coeff = (cos(angle / 2), sin(angle / 2) * axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]);
}

template <typename T>
inline Quaternion<T> Quaternion<T>::operator-() const
{
	return Quaternion<T>(-coeff);
}


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
inline Quaternion<T> Quaternion<T>::operator*(UnitQuaternion<T> &q1) const
{
	return Quaternion<T>(coeff * q1.coeff);
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
	
	std::cout << conjugate() << std::endl;
	return sqrt(coeff.dot(coeff));
}

template <typename T>
inline T Quaternion<T>::dot(Quaternion<T> &q1) const
{
	return coeff.dot(coeff);
}

template <typename T>
inline UnitQuaternion<T> Quaternion<T>::normalize() const
{
	return UnitQuaternion<T>(coeff / norm());
}

template <typename T>
inline Quaternion<T> Quaternion<T>::inv() const
{
	return this->conjugate() / pow(this->norm(), 2);
}

template <typename T>
inline T UnitQuaternion<T>::getAngle() const
{
	return 2 * acos(this->coeff[0]);
}

template <typename T>
inline cv::Vec<T, 3> UnitQuaternion<T>::getAxis() const
{
	T angle = getAngle();
	cv::Vec<T, 4> coeff = this->coeff;
	cv::Vec<T, 3> axis;
	if (abs(sin(angle)) < 0.0001)
		return axis = {0, 0 ,0}; // TBD	
	axis[0] = coeff[1] / sin(angle);
	axis[1] = coeff[2] / sin(angle);
	axis[2] = coeff[3] / sin(angle);
	return axis;
}
/*
template <typename T>
cv::Mat UnitQuaternion<T>::getRotMat(const T &angle, const cv::Vec<T, 3> &axis)
{
	// assert(isNormalized() == true);
	// if (Quaternion<T>::isNormalized(axis) == false)
	// {
	//     throw("balabala")
	// }
	T a = cos(angle / 2), b = sin(angle / 2) * axis[0], c = sin(angle / 2) * axis[1], d = sin(angle / 2) * axis[2];
	return UnitQuaternion<T>(a, b, c, d).toRotMat33();
}
*/
template <typename T>
cv::Mat UnitQuaternion<T>::toRotMat44()
{
	cv::Vec<T, 4> coeff = this->coeff;
	T a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];
	cv::Matx<T, 4, 4> R{
		 1, 0,                       0,                       0,
		 0, 1 - 2 * (c * c + d * d), 2 * (b * c - a * d),     2 * (a * c + b * d),
		 0, 2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * b),
		 0, 2 * (b * d - a * c)    , 2 * (a * b + c * d),     1 - 2 * (b * b + c * c)};
	return cv::Mat(R); 
}

template <typename T>
cv::Mat UnitQuaternion<T>::toRotMat33()
{
	cv::Vec<T, 4> coeff = this->coeff;
	T a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];
	cv::Matx<T, 3, 3> R{
		  1 - 2 * (c * c + d * d), 2 * (b * c - a * d),     2 * (a * c + b * d),
		  2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * b),
		  2 * (b * d - a * c)    , 2 * (a * b + c * d),     1 - 2 * (b * b + c * c)};
	return cv::Mat(R); 
}

template <typename T>
inline cv::Vec<T, 4> Quaternion<T>::getCoeff() const
{
	return coeff;
}

/*
template <typename T>
void UnitQuaternion<T>::transform(cv::Mat &beTransed, const T &angle, const cv::Vec<T, 3> &axis)
{
	cv::Mat rotateMat = getRotMat(angle, axis); 
	beTransed = rotateMat * beTransed.t();
	beTransed = beTransed.t();
}
*/

template <typename T>
Quaternion<T> UnitQuaternion<T>::lerp(const UnitQuaternion<T> &q1, const UnitQuaternion<T> &q2, T t)
{
	return (1 - t) * q1 + t * q2;
}

template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::slerp(UnitQuaternion<T> &q1, UnitQuaternion<T> &q2, T t)
{
	T cosTheta = q1.dot(q2);
	if (cosTheta < 0)
		q1 = -q1;
	else if (angle > 0.999) // TBD
		return nlerp(q1, q2, t);
	T sinTheta = sqrt(1 - cosTheta * cosTheta);
	T angle = atan2(sinTheta, cosTheta);
	return sin((1 - t) * angle) * q1 / sinTheta + sin(t * angle) / sinTheta * q2;
}

template <typename T>
inline UnitQuaternion<T> UnitQuaternion<T>::nlerp(const UnitQuaternion<T> &q1, const UnitQuaternion<T> &q2, T t)
{
	return ((1 - t) * q1 + t * q2).normalize();
}


template <typename T>
inline bool Quaternion<T>::isNormal() const
{
	double eps = 0.00001;
	double normVar = sqrt(coeff.dot(coeff));
	if ((normVar > 1 - eps) && (normVar < 1 + eps))
		return true;
	return false;
}


template <typename T>
UnitQuaternion<T>::UnitQuaternion(){}

template <typename T>
UnitQuaternion<T>::UnitQuaternion(cv::Mat &R)
{
	assert(R.rows == 3 && R.cols == 3);
	T qw, qx, qy, qz, S;
	T trace = R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2);
	if (trace > 0.00000001)
	{
		S = sqrt(trace + 1) * 2; // TBD
		qx = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
		qy = (R.at<T>(2, 0) - R.at<T>(0, 2)) / S;
		qz = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
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
			qw = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
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
			qw = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
		}
	}
	this->coeff = {qw, qx, qy, qz}; 
}

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const Quaternion<T> &q){
	this->coeff = q.normalize().coeff;
}

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const UnitQuaternion<T> &q){
	this->coeff = q.normalize().coeff;
}

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const cv::Vec<T, 4> &coeff)
{
	// assert(coeff.dot(coeff) == 1);
	std::cout << "Vec constructor here" << std::endl;
	//this->coeff = Quaternion<T>(coeff).normalize().coeff;
	this->coeff = coeff;
}	

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const T angle, cv::Vec<T, 3> &axis)
{
	this->coeff = {cos(angle / 2), sin(angle / 2) * axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]};
}		

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const T qw, const T qx, const T qy, const T qz){
	this->coeff = Quaternion<T>(qw, qx, qy, qz).normalize().coeff;
}

template <typename T>
Quaternion<T> UnitQuaternion<T>::log(UnitQuaternion<T> &q1)
{
	cv::Vec<T, 4> coeff = q1.getCoeff();
	T alpha = acos(coeff[0]);
	if (abs(sin(alpha)) < 0.00001)
	{
		return Quaternion<T>(0, coeff[1], coeff[2], coeff[3]); //TBD
	}
	T k = alpha / sin(alpha);
	return Quaternion<T>(0, coeff[1] * k, coeff[2] * k, coeff[3] * k);
}

template <typename T>
UnitQuaternion<T> Quaternion<T>::exp(Quaternion<T> &q1)
{
	cv::Vec<T, 4> coeff = q1.getCoeff();
	T alpha = sqrt(coeff.dot(coeff));
	T k = abs(alpha) < 0.00001 ? 1 : (sin(alpha) / alpha);  //TBD
	UnitQuaternion<T> q2(cos(alpha), coeff[1] * k, coeff[2] * k, coeff[3] * k);
	return q2;
}

template <typename T>
inline Quaternion<T> UnitQuaternion<T>::squad(const UnitQuaternion<T> &q1, const UnitQuaternion<T> &q2,
										  	  const UnitQuaternion<T> &q3, const UnitQuaternion<T> &q4, T t)
{
	return slerp(slerp(q1, q4, t), slerp(q2, q3, t), 2 * t * (1 - t));
}

template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::interPoint(const UnitQuaternion<T> &q0,
									 			const UnitQuaternion<T> &q1,
									 			const UnitQuaternion<T> &q2) const
{
	Quaternion<T> qconj = q1.conjugate();
	Quaternion<T> c1 = log(qconj * q0);
	Quaternion<T> c2 = log(qconj * q2);
	UnitQuaternion<T> res =  q1 * exp((- (c1 + c2) / 4)).normalize();
	return res;
}

template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::spline(const UnitQuaternion<T> &q0, const UnitQuaternion<T> &q1, const UnitQuaternion<T> &q2, const UnitQuaternion<T> &q3, T t) const
{
	T angle;
	for (auto &i: {q0, q1, q2})
	{
		angle = sqrt(acos(q3.dot(i)));
		if (angle > CV_PI / 2)
			i = -i;
	}
	UnitQuaternion<T> s1 = interPoint(q0, q1, q2);
	UnitQuaternion<T> s2 = interPoint(q1, q2, q3);
	return squad(q1, s1, s2, q2, t);
}

int main(){
	// test_operator();
	// test_unit_basic();
	cv::Vec<double, 4> a{1,2,3,4};
	Quaternion<double> q(1,2,3,4);
	q.norm();
	// q.normalize();
	return 0;
}
