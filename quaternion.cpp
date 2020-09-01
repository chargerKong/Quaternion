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
Quaternion<T>::Quaternion(cv::Mat &R)
{
	// 确定Mat是什么类型
	assert(R.rows == 3 && R.cols == 3);
	T qw, qx, qy, qz, S;
	T trace = R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2);
	if (trace > 0.00000001)
	{
		S = sqrt(trace) * 2; // TBD
		qx = -(R.at<T>(2, 1) - R.at<T>(1, 2)) / S;
		qy = -(R.at<T>(0, 2) - R.at<T>(2, 0)) / S;
		qz = -(R.at<T>(1, 0) - R.at<T>(0, 1)) / S;
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
			//qx = -0.25 * S;
			//qy = -(R.at<T>(1, 0) + R.at<T>(0, 1)) / S;
			//qz = -(R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
			//qw = (R.at<T>(2, 1) - R.at<T>(1, 2)) / S;
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
	coeff = {qw, -qx, -qy, -qz}; 
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
inline T Quaternion<T>::getAngle() const
{
	return 2 * acos(coeff[0]);
}

template <typename T>
inline cv::Vec<T, 3> Quaternion<T>::getAxis() const
{
	T angle = getAngle();
	cv::Vec<T, 3> axis;
	axis[0] = coeff[1] / sin(angle / 2);
	axis[1] = coeff[2] / sin(angle / 2);
	axis[2] = coeff[3] / sin(angle / 2);
	return axis;
}

/*
template <typename T>
inline Quaternion<T> Quaternion<T>::t() const
{
	std::cout << coeff.t() << std::endl;
	return Quaternion(coeff.t());
}
*/

template <typename T>
cv::Mat Quaternion<T>::getRotMat(const T &angle, const cv::Vec<T, 3> &axis)
{
	assert(Quaternion<T>::isNormalized(axis) == true);
	// if (Quaternion<T>::isNormalized(axis) == false)
	// {
	//     throw("balabala")
	// }
	T a = cos(angle / 2), b = sin(angle / 2) * axis[0], c = sin(angle / 2) * axis[1], d = sin(angle / 2) * axis[2];
	return Quaternion<T>(a, b, c, d).toRotMat();
}
	
template <typename T>
cv::Mat Quaternion<T>::toRotMat()
{
	assert(Quaternion<T>::isNormalized(coeff) == true);
	T a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];
	cv::Matx<T, 4, 4> R{
		 1, 0,                       0,                       0,
		 0, 1 - 2 * (c * c + d * d), 2 * (b * c - a * d),     2 * (a * c + b * d),
		 0, 2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * b),
		 0, 2 * (b * d - a * c)    , 2 * (a * b + c * d),     1 - 2 * (b * b + c * c)};
	return cv::Mat(R); 
}

/*
template <typename T>
template <typename _T>
_T Quaternion<T>::transform(_T &beTransed, const T &angle, const cv::Vec<T, 3> &axis)
{
	//Todo:: 只修改beTransed,  不复制返回
	cv::Mat rotateMat = getRotMat(angle, axis); 
	beTransed = rotateMat * beTransed.t();
	std::cout << beTransed.t() << std::endl;
	return beTransed.t();
}
*/

template <typename T>
void Quaternion<T>::transform(cv::Mat &beTransed, const T &angle, const cv::Vec<T, 3> &axis)
{
	//Todo:: 只修改beTransed,  不复制返回
	cv::Mat rotateMat = getRotMat(angle, axis); 
	beTransed = rotateMat * beTransed.t();
	beTransed = beTransed.t();
}


template <typename T>
Quaternion<T> Quaternion<T>::slerp(Quaternion<T> &q1, Quaternion<T> &q2, T t)
{
	T angle = acos(q1.dot(q2));
	if (angle > CV_PI)
		q1 = -q1;
	else if (angle < CV_PI / 180)
		return nlerp(q1, q2, t);
	return sin((1-t) * angle) * q1 / sin(angle) + sin(t * angle) / sin(angle) * q2;
}

template <typename T>
inline Quaternion<T> Quaternion<T>::nlerp(Quaternion<T> &q1, Quaternion<T> &q2, T t)
{
	return ((1 - t) * q1 + t * q2).normalize();
}

template <typename T>
inline Quaternion<T> Quaternion<T>::getRotQuat(const T& angle, const cv::Vec<T, 3> &axis)
{
	assert(Quaternion<T>::isNormalized(axis) == true);
	// if (Quaternion<T>::isNormalized(axis) == false)
	// {
	//     raise("balabala")
	// }
	return Quaternion(cos(angle / 2), sin(angle / 2) * axis[0], sin(angle / 2) * axis[1], sin((angle / 2)) * axis[2]);
}

template <typename T>
template <typename _Tp>
bool Quaternion<T>::isNormalized(_Tp obj)
{
	double eps = 0.00001;
	double normVar = sqrt(obj.dot(obj));
	if ((normVar > 1 - eps) && (normVar < 1 + eps))
		return true;
	return false;
}





int main(){
	test_operator();
	std::cout << "=================================" << std::endl;
	Quaternion<float> q1{2,1,3,4};
	q1 = q1.normalize();
	assert(q1.norm() == 1);
	bool ans = Quaternion<double>::isNormalized(cv::Vec<double, 4>{1,2,3,4});
	assert(ans == false);
	ans = Quaternion<double>::isNormalized(Quaternion<double>{1.1,2,3,4});
	assert(ans == false);
	ans = Quaternion<double>::isNormalized(Quaternion<double>{1,2,3,4} / sqrt(30));
	assert(ans == true);
	ans = Quaternion<double>::isNormalized(cv::Vec<double, 4>{1,2,3,4} / sqrt(30));
	assert(ans == true);
	double angle = 3.1415926 / 3;
	cv::Vec<double, 3> axis{1/sqrt(3),1/sqrt(3),1/sqrt(3)};	
	
	
	cv::Mat testMat = (cv::Mat_<double>(2,4) << 0,1,1,1,0,4,2,3);
	Quaternion<double>::transform(testMat, angle, axis);
	std::cout << "after transformation:\n" << testMat << std::endl;
	Quaternion<double> rotateQuat = Quaternion<double>::getRotQuat(angle, axis);
	Quaternion<double> betransed1(0,1,1,1);
	Quaternion<double> betransed2(0,4,2,3);
	
	std::cout << "One point[0,1,1,1]:" << rotateQuat * betransed1 * rotateQuat.conjugate()<<std::endl; 	
	std::cout << "One point[0,4,2,3]:" << rotateQuat * betransed2 * rotateQuat.conjugate()<<std::endl; 	
	
	// have some problem
	Quaternion<double> q(1,-4,-2,-3);
	q = q.normalize();
	std::cout << "\nQuat2mat: Quat:\n" << q <<"\nMat:\n" << q.toRotMat() << std::endl;
	testMat = q.toRotMat().colRange(1, 4).clone().rowRange(1, 4).clone();
	// testMat = (cv::Mat_<double>(3,3) << -39,12,24,12,-49,16,24,16,-25);
	Quaternion<double> q2(testMat);
	std::cout <<"\nMat2Quat：Mat:\n" << testMat << "\nQuat:" << q2 << std::endl;

	// test angle and axis calculation
	Quaternion<double> q3(Quaternion<double>::getRotQuat(angle, axis));
	std::cout << "\nOne Quaternion" << q3 << " is creat by \nangle:" << angle << "\naxis:" << axis << std::endl;
	std::cout << "after get Angle and getAxis: \nangle:" << q3.getAngle() << "\naxis:" << q3.getAxis() << std::endl;
	//assert(angle == q3.getAngle());  // have little difference
	axis = {0, 0, 1};
	double t = 0.5;
	Quaternion<double> q4(Quaternion<double>::getRotQuat(0, axis));
	Quaternion<double> q5(Quaternion<double>::getRotQuat(3.1415926, axis));
	Quaternion<double> q6(Quaternion<double>::slerp(q4, q5, t));
	std::cout << "\nslerp between q4: " << q4 << "q5: " << q5 << " with t=" << t << "\nq6:" << q6 << std::endl;
	
	
	return 0;

}
