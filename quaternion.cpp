#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif

#include <assert.h>
//#nclude <iostream>
#include <math.h>
//#include "test/test_basic.cpp"
#include "test/test_unit_basic.cpp"
//using namespace cv;

template <typename T>
Quaternion<T>::Quaternion(cv::Vec<T, 4> coeff):coeff(coeff){}

template <typename T>
Quaternion<T>::Quaternion(T qw, T qx, T qy, T qz):coeff(qw, qx, qy, qz){}


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
	return Quaternion<T>(coeff * q1.getCoeff());
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
	std::cout << ":Quaternion" << std::endl;
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

/*
template <typename T>
inline T Quaternion<T>::dot(Quaternion<T> &q) const
{
	return coeff.dot(q.coeff);
} 
*/

template <typename T>
template <typename _T>
inline T Quaternion<T>::dot(_T &q1, _T &q2)
{
	return q1.getCoeff().dot(q2.getCoeff());
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
cv::Mat UnitQuaternion<T>::getRotMat(const T &angle, const cv::Vec<T, 3> &axis)
{
	assert(isNormalized(axis) == true);
	// if (Quaternion<T>::isNormalized(axis) == false)
	// {
	//     throw("balabala")
	// }
	T a = cos(angle / 2), b = sin(angle / 2) * axis[0], c = sin(angle / 2) * axis[1], d = sin(angle / 2) * axis[2];
	return UnitQuaternion<T>(a, b, c, d).toRotMat();
}
	
template <typename T>
cv::Mat UnitQuaternion<T>::toRotMat44()
{
	assert(isNormalized(coeff) == true);
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
	assert(Quaternion<T>::isNormalized(coeff) == true);
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
Quaternion<T> Quaternion<T>::lerp(const Quaternion<T> &q1, const Quaternion<T> &q2, T t)
{
	return (1 - t) * q1 + t * q2;
}

template <typename T>
Quaternion<T> Quaternion<T>::slerp(Quaternion<T> &q1, Quaternion<T> &q2, T t)
{
	// q1 and q2 is unit quaternion
	T angle = acos(q1.dot(q2));
	if (angle > CV_PI)
		q1 = -q1;
	else if (angle < CV_PI / 180) // TBD
		return nlerp(q1, q2, t);
	return sin((1 - t) * angle) * q1 / sin(angle) + sin(t * angle) / sin(angle) * q2;
}

template <typename T>
inline Quaternion<T> Quaternion<T>::nlerp(const Quaternion<T> &q1, const Quaternion<T> &q2, T t)
{
	return ((1 - t) * q1 + t * q2).normalize();
}


template <typename T>
inline Quaternion<T> Quaternion<T>::squad(const Quaternion<T> &q1, const Quaternion<T> &q2,
										  const Quaternion<T> &q3, const Quaternion<T> &q4, T t)
{
	return slerp(slerp(q1, q4, t), slerp(q2, q3, t), 2* t * (1 - t));
}

template <typename T>
inline Quaternion<T> Quaternion<T>::getRotQuat(const T& angle, const cv::Vec<T, 3> &axis)
{
	assert(isNormalized(axis) == true);
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
	coeff = {qw, qx, qy, qz}; 
}

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const Quaternion<T> &q):coeff(q.normalize().coeff){}


template <typename T>
UnitQuaternion<T>::UnitQuaternion(const UnitQuaternion<T> &q):coeff(q.coeff){std::cout<<"copy constuctor"<<std::endl;}

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const cv::Vec<T, 4> &coeff):coeff(coeff){}
/*{
	// assert(coeff.dot(coeff) == 1);
		std::cout << "Vec constructor here" << std::endl;
	//	throw "vector must be normalized";
	this->coeff = coeff;
}*/	

template <typename T>
UnitQuaternion<T>::UnitQuaternion(const T &angle, cv::Vec<T, 3> &axis)
{
	// std::cout << axis.dot(axis)<<std::endl;
	// assert(axis.dot(axis) == 1.0); 会报错
	// throw "axis must be normalized";
	coeff = {cos(angle / 2), sin(angle / 2) * axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]};
}		

template <typename T>
UnitQuaternion<T>::UnitQuaternion(T qw, T qx, T qy, T qz):coeff(qw, qx, qy, qz){}

>>>>>>> f3b406c4eb5579992de33388eecfbe092f9f5901

template <typename T>
cv::Vec<T, 4> UnitQuaternion<T>::getCoeff()
{
	return coeff;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const UnitQuaternion<T> &q)
{
	os << q.coeff;
	return os;
}

template <typename T>
inline UnitQuaternion<T> UnitQuaternion<T>::operator-() const
{
	return UnitQuaternion<T>(-coeff);
}


template <typename T>
inline bool UnitQuaternion<T>::operator==(const UnitQuaternion<T> &q) const
{
	return coeff == q.coeff;
}

template <typename T>
inline UnitQuaternion<T> UnitQuaternion<T>::operator+(const UnitQuaternion<T> &q1) const
{
	//return UnitQuaternion<T>(*this) += q1; //error
	//cv::Vec<T, 4> coe{q1.coeff + this->coeff};
	//UnitQuaternion<T> q(coe);
	// UnitQuaternion<T> q(q1.coeff + this->coeff); //error
	//return q;
	
	return UnitQuaternion<T>(coeff + q1.coeff); //error

}

template <typename T>
inline UnitQuaternion<T>& UnitQuaternion<T>::operator+=(const UnitQuaternion<T> &q1)
{
	coeff += q1.coeff;
	return *this;
}

template <typename T>
inline UnitQuaternion<T> UnitQuaternion<T>::operator-(const UnitQuaternion<T> &q1) const
{
	return UnitQuaternion<T>(coeff - q1.coeff);
}


template <typename T>
inline UnitQuaternion<T>& UnitQuaternion<T>::operator-=(const UnitQuaternion<T> &q1)
{
	coeff -= q1.coeff;
	return *this;
}


template <typename T>
inline UnitQuaternion<T> UnitQuaternion<T>::operator*(const UnitQuaternion<T> &q1) const
{
	/*cv::Vec<T, 4> vec{coeff * q1.coeff};
	UnitQuaternion<T> q(vec);
	return q;
	*/
	return UnitQuaternion<T>(coeff * q1.coeff);
}

template <typename T>
inline Quaternion<T> UnitQuaternion<T>::operator*(Quaternion<T> &q1) const
{
	return Quaternion<T>(coeff * q1.getCoeff());


template <typename T>
UnitQuaternion<T> operator*(const UnitQuaternion<T> &q1, const T a)
{
	/*
	cv::Vec<T, 4> vec{q1.coeff * a};
	UnitQuaternion<T> q(vec);
	return q;
	*/
	return UnitQuaternion<T>(q1.coeff * a);
}

template <typename T>
UnitQuaternion<T> operator*(const T a, const UnitQuaternion<T> &q1)
{
	/*
	cv::Vec<T, 4> vec{q1.coeff * a};
	UnitQuaternion<T> q(vec);
	return q;!>!/
	*/
	return q1 * a; 
}

template <typename T>
inline UnitQuaternion<T>& UnitQuaternion<T>::operator*=(const UnitQuaternion<T> &q1)
{
	coeff *= q1.coeff;
	return *this;
}

template <typename T>
UnitQuaternion<T>& UnitQuaternion<T>::operator*=(const T &q1)
{
	coeff *= q1;
	return *this;
}

template <typename T>
inline UnitQuaternion<T>& UnitQuaternion<T>::operator/=(const T &a)
{
	coeff /= a;
	return *this;
}

template <typename T>
inline UnitQuaternion<T> UnitQuaternion<T>::operator/(const T &a) const
{
	return UnitQuaternion<T>(coeff / a);
}

template <typename T>
inline const T& UnitQuaternion<T>::operator[](std::size_t n) const
{
	return coeff[n];
}

template <typename T>
inline T& UnitQuaternion<T>::operator[](std::size_t n)
{
	return coeff[n];
}

		
template <typename T>
Quaternion<T> UnitQuaternion<T>::log() const
{
	T alpha = acos(coeff[0]);
	if (abs(sin(alpha)) < 0.00001)
	{
		return Quaternion<T>(0, coeff[1], coeff[2], coeff[3]);
	}
	T k = alpha / sin(alpha);
	return Quaternion<T>(0, coeff[1] * k, coeff[2] * k, coeff[3] * k);
}

template <typename T>
UnitQuaternion<T> Quaternion<T>::exp() const
{
	T alpha = sqrt(coeff.dot(coeff));
	T k = abs(alpha) < 0.00001 ? 1 : (sin(alpha) / alpha);
	return UnitQuaternion<T>(cos(alpha), coeff[1] * k, coeff[2] * k, coeff[3] * k);
}

template <typename T>
inline Quaternion<T> UnitQuaternion<T>::squad(const Quaternion<T> &q1, const Quaternion<T> &q2,
										  	  const Quaternion<T> &q3, const Quaternion<T> &q4, T t)
{
	return slerp(slerp(q1, q4, t), slerp(q2, q3, t), 2 * t * (1 - t));
}

template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::interPoint(const UnitQuaternion<T> &q0,
									 			const UnitQuaternion<T> &q1,
									 			const UnitQuaternion<T> &q2) const
{
	Quaternion<T> qconj = q1.conjugate();
	Quaternion<T> c1 = (qconj * q0).log();
	Quaternion<T> c2 = (qconj * q2).log();
	return q1 * (- (c1 + c2) / 4).exp().normalize();
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
	//test_operator();
	test_unit_basic();
	//UnitQuaternion<double> q1(1,2,3,4);
	
	return 0;
}
