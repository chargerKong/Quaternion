#ifndef QUATERNION_H
#define QUATERNION_H
#include "quaternion.hpp"
#endif
//using namespace cv;
#include <assert.h>
//#nclude <iostream>
#include <math.h>
// using namespace cv;

#define EPS 0.0001

template <typename T>
Quat<T>::Quat(cv::Vec<T, 4> coeff):w(coeff[0]), x(coeff[1]), y(coeff[2]), z(coeff[3]){}

template <typename T>
Quat<T>::Quat(T qw, T qx, T qy, T qz):w(qw), x(qx), y(qy), z(qz){}

template <typename T>
Quat<T>::Quat(const T angle, const cv::Vec<T, 3> &axis)
{
	w = cos(angle / 2);
	x = sin(angle / 2) * axis[0];
	y = sin(angle / 2) * axis[1];
	z = sin(angle / 2) * axis[2];
}

template <typename T>
Quat<T>::Quat(const cv::Mat &R)
{
	assert(R.rows == 3 && R.cols == 3);
	T S;
	T trace = R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2);
	if (trace > 0.00000001)
	{
		S = sqrt(trace + 1) * 2; // TBD
		x = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
		y = (R.at<T>(2, 0) - R.at<T>(0, 2)) / S;
		z = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
		w = 0.25 * S;
	}
	else
	{
		if (R.at<T>(0, 0) > R.at<T>(1, 1) && R.at<T>(0, 0) > R.at<T>(2, 2))
		{
			S = sqrt(1.0 + R.at<T>(0, 0) - R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
			x = 0.25 * S;
			y = (R.at<T>(1, 0) + R.at<T>(0, 1)) / S;
			z = (R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
			w = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
		}
		else if (R.at<T>(1, 1) > R.at<T>(2, 2))
		{
			S = sqrt(1.0 - R.at<T>(0, 0) + R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
			x = (R.at<T>(0, 1) + R.at<T>(1, 0)) / S; 
			y = 0.25 * S;
			z = (R.at<T>(1, 2) + R.at<T>(2, 1)) / S;
			w = (R.at<T>(0, 2) - R.at<T>(2, 0)) / S;
		}
		else
		{
			S = sqrt(1.0 - R.at<T>(0, 0) - R.at<T>(1, 1) + R.at<T>(2, 2)) * 2;
			x = (R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
			y = (R.at<T>(1, 2) + R.at<T>(2, 1)) / S;
			z = 0.25 * S;
			w = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
		}
	}
}

template <typename T>
inline Quat<T> Quat<T>::operator-() const
{
	return Quat<T>(-w, -x, -y, -z);
}


template <typename T>
inline bool Quat<T>::operator==(const Quat<T> &q) const
{
	return (w == q.w && x == q.x && y == q.y && z == q.z);
}

template <typename T>
inline Quat<T> Quat<T>::operator+(const Quat<T> &q1) const
{
	return Quat<T>(w + q1.w, x + q1.x, y + q1.y, z + q1.z);
}

template <typename T>
inline Quat<T> Quat<T>::operator-(const Quat<T> &q1) const
{
	return Quat<T>(w - q1.w, x - q1.x, y - q1.y, z - q1.z);
}

template <typename T>
inline Quat<T>& Quat<T>::operator+=(const Quat<T> &q1)
{
	w += q1.w;
	x += q1.x;
	y += q1.y;
	z += q1.z;
	return *this;
}

template <typename T>
inline Quat<T>& Quat<T>::operator-=(const Quat<T> &q1)
{
	w -= q1.w;
	x -= q1.x;
	y -= q1.y;
	z -= q1.z;
	return *this;
}

template <typename T>
inline Quat<T> Quat<T>::operator*(const Quat<T> &q1) const
{
	// T a = q1.w, b = q1.x, c = q1.y, d = q1.z;
	// cv::Mat mat = (cv::Mat_<T>(4, 4) << a, -b, -c, -d,
	//									b,  a,  d, -c,
	//									c, -d, 	a,  b,
	//
	//d,  c, -b,  a);
	cv::Vec<T, 4> q{w, x, y, z};
	cv::Vec<T, 4> q2{q1.w, q1.x, q1.y, q1.z}; 
	return Quat<T>(q * q2);
}


template <typename T>
Quat<T> operator*(const Quat<T> &q1, const T a)
{
	return Quat<T>(a * q1.w, a * q1.x, a * q1.y, a * q1.z);
}

template <typename T>
Quat<T> operator*(const T a, const Quat<T> &q1)
{
	return Quat<T>(a * q1.w, a * q1.x, a * q1.y, a * q1.z);
}

template <typename T>
inline Quat<T>& Quat<T>::operator*=(const Quat<T> &q1)
{
	T qw, qx, qy, qz;
	qw = w * q1.w - x * q1.x - y * q1.y - z * q1.z;
	qx = x * q1.w + w * q1.x + y * q1.z - z * q1.y;
	qy = y * q1.w + w * q1.y + z * q1.x - x * q1.z;
	qz = z * q1.w + w * q1.z + x * q1.y - y * q1.x;
	w = qw;
	x = qx;
	y = qy;
	z = qz;
	return *this;
}

template <typename T>
Quat<T>& Quat<T>::operator*=(const T &q1)
{
	w *= q1;
	x *= q1;
	y *= q1;
	z *= q1;
	return *this;
}

template <typename T>
inline Quat<T>& Quat<T>::operator/=(const T &a)
{
	w /= a;
	x /= a;
	y /= a;
	z /= a;
	return *this;
}

template <typename T>
inline Quat<T> Quat<T>::operator/(const T &a) const
{
	return Quat<T>(w / a, x / a, y / a, z / a);
}

template <typename T>
inline const T& Quat<T>::operator[](std::size_t n) const
{
	switch (n) {
		case 0:
			return w;
			break;
		case 1:
			return x;
			break;
		case 2:
			return y;
			break;
		case 3:
			return z;
			break;
		default:
			throw "subscript exceeds the index range";
	}
}

template <typename T>
inline T& Quat<T>::operator[](std::size_t n)
{	
	switch (n) {
		case 0:
			return w;
			break;
		case 1:
			return x;
			break;
		case 2:
			return y;
			break;
		case 3:
			return z;
			break;
		default:
			throw "subscript exceeds the index range";
	}
}

template <typename T>
std::ostream & std::operator<<(std::ostream &os, const Quat<T> &q)
{
	os << cv::Vec<T, 4>{q.w, q.x, q.y, q.z};
	return os;
}

template <typename T>
inline Quat<T> Quat<T>::conjugate() const
{
	return Quat<T>(w, -x, -y, -z);
}

template <typename T>
inline T Quat<T>::norm() const
{
	return sqrt(dot(*this));
}


template <typename T>
inline Quat<T> power(Quat<T> &q1, T x)
{
	T angle = q1.getAngle();
	cv::Vec<T, 3> axis = q1.getAxis();
	return pow(q1.norm(), x) * Quat<T>(x * angle, axis);
}

template <typename T>
inline Quat<T> power(Quat<T> &p, Quat<T> &q)
{
	return exp(p * log(q));
}

template <typename T>
inline T Quat<T>::dot(Quat<T> q1) const
{
	return w * q1.w + x * q1.x + y * q1.y + z * q1.z;
}

template <typename T>
inline cv::Vec<T, 3> Quat<T>::crossProduct(Quat<T> q1) const
{
	return cv::Vec<T, 3>{y * q1.z - z * q1.y, z * q1.x - x * q1.y, x * q1.y - y * q1.x};
}

template <typename T>
inline Quat<T> Quat<T>::normalize() const
{
	return Quat<T>(w / norm(), x / norm(), y / norm(), z / norm()) ;
}

template <typename T>
inline Quat<T> Quat<T>::inv() const
{
	return this->conjugate() / pow(this->norm(), 2);
}

template <typename T>
inline Quat<T> Quat<T>::sinh() const
{
	cv::Vec<T, 3> v{x, y ,z};
	T vNorm = sqrt(v.dot(v));
	T k = cosh(w) * sin(v.norm()) / vNorm;
	return Quat<T>(sinh(w) * cos(vNorm), v[0] * k, v[1] * k, v[2] * k);
}

template <typename T>
inline Quat<T> Quat<T>::cosh() const
{
	cv::Vec<T, 3> v{x, y ,z};
	T vNorm = sqrt(v.dot(v));
	T k = sinh(w) * sin(v.norm()) / vNorm;
	return Quat<T>(cosh(w) * cos(vNorm), v[0] * k, v[1] * k, v[2] * k);
}


template <typename T>
inline Quat<T> Quat<T>::tanh() const
{
	return sinh() * cosh().inv();
}

template <typename T>
inline Quat<T> Quat<T>::sin() const
{
	cv::Vec<T, 3> v{x, y ,z};
	T vNorm = sqrt(v.dot(v));
	T k = cos(w) * sinh(v.norm()) / vNorm;
	return Quat<T>(sin(w) * cosh(vNorm), v[0] * k, v[1] * k, v[2] * k);
}

template <typename T>
inline Quat<T> Quat<T>::cos() const
{
	cv::Vec<T, 3> v{x, y ,z};
	T vNorm = sqrt(v.dot(v));
	T k = sin(w) * sinh(v.norm()) / vNorm;
	return Quat<T>(cos(w) * cosh(vNorm), -v[0] * k, -v[1] * k, -v[2] * k);
}

template <typename T>
inline Quat<T> Quat<T>::tan() const
{
	return sin() / cos().inv();
}

template <typename T>
inline Quat<T> Quat<T>::asinh() const
{
	return log(*this + pow(*this * *this + 1, 0.5));
}

template <typename T>
inline Quat<T> Quat<T>::acosh() const
{
	return log(*this + pow(*this * *this - 1, 0.5));
}

template <typename T>
inline Quat<T> Quat<T>::atanh() const
{
	return 1/2 * log(1 + *this) - log(1 - *this);
}

template <typename T>
inline T Quat<T>::getAngle() const
{
	return 2 * acos(w / norm());
}

template <typename T>
inline cv::Vec<T, 3> Quat<T>::getAxis() const
{
	T angle = getAngle();
	if (abs(sin(angle / 2)) < EPS)
		return cv::Vec<T, 3> {x, y, z}; // TBD	
	return cv::Vec<T, 3> {x, y, z} / (norm() * sin(angle / 2));
}
/*
template <typename T>
cv::Mat UnitQuat<T>::getRotMat(const T &angle, const cv::Vec<T, 3> &axis)
{
	// assert(isNormalized() == true);
	// if (Quat<T>::isNormalized(axis) == false)
	// {
	//     throw("balabala")
	// }
	T a = cos(angle / 2), b = sin(angle / 2) * axis[0], c = sin(angle / 2) * axis[1], d = sin(angle / 2) * axis[2];
	return UnitQuat<T>(a, b, c, d).toRotMat33();
}
*/
template <typename T>
cv::Mat Quat<T>::toRotMat44() const
{
	T dotVal = dot(*this);
	cv::Matx<T, 4, 4> R{
		 dotVal, 0                           , 0                           , 0,
		 0     , dotVal - 2 * (y * y + z * z), 2 * (x * y - w * z)         , 2 * (w * y + x * z),
		 0     , 2 * (x * y + w * z)         , dotVal - 2 * (x * x + z * z), 2 * (y * z - w * x),
		 0     , 2 * (x * z - w * y)         , 2 * (w * x + y * z)         , dotVal - 2 * (x * x + y * y)};
	return cv::Mat(R); 
}

template <typename T>
cv::Mat Quat<T>::toRotMat33() const
{
	if (!isNormal())
		throw "Quaternion must be normalized";
	cv::Matx<T, 3, 3> R{
		  1 - 2 * (y * y + z * z), 2 * (x * y - w * z)    , 2 * (w * y + x * z),
		  2 * (x * y + w * z)    , 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
		  2 * (x * z - w * y)    , 2 * (w * x + y * z)    , 1 - 2 * (x * x + y * y)};
	return cv::Mat(R); 
}

template <typename T>
cv::Vec<T, 4> Quat<T>::toVec() const
{
	return cv::Vec<T, 4>{w, x, y, z};
}


/*
template <typename T>
void UnitQuat<T>::transform(cv::Mat &beTransed, const T &angle, const cv::Vec<T, 3> &axis)
{
	cv::Mat rotateMat = getRotMat(angle, axis); 
	beTransed = rotateMat * beTransed.t();
	beTransed = beTransed.t();
}
*/

template <typename T>
Quat<T> Quat<T>::lerp(const Quat<T> &q1, const Quat<T> &q2, T t)
{
	return (1 - t) * q1 + t * q2;
}

template <typename T>
Quat<T> Quat<T>::slerp(const Quat<T> &q1, const Quat<T> &q2, T t)
{
	T cosTheta = q1.dot(q2);
	if (cosTheta < 0)
		q1 = -q1;
	else if (cosTheta > 0.999) // TBD
		return nlerp(q1, q2, t);
	T sinTheta = sqrt(1 - cosTheta * cosTheta);
	T angle = atan2(sinTheta, cosTheta);
	return sin((1 - t) * angle) * q1 / sinTheta + sin(t * angle) / sinTheta * q2;
}

template <typename T>
inline Quat<T> Quat<T>::nlerp(const Quat<T> &q1, const Quat<T> &q2, T t)
{
	return ((1 - t) * q1 + t * q2).normalize();
}


template <typename T>
inline bool Quat<T>::isNormal() const
{
	double normVar = norm();
	if ((normVar > 1 - EPS) && (normVar < 1 + EPS))
		return true;
	return false;
}

/*
template <typename T>
Quat<T>::Quat(){}

template <typename T>
Quat<T>::Quat(cv::Mat &R)
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
Quat<T>::Quat(const Quat<T> &q){
	this->coeff = q.normalize().coeff;
}

template <typename T>
Quat<T>::Quat(const Quat<T> &q){
	this->coeff = q.normalize().coeff;
}

template <typename T>
Quat<T>::Quat(const cv::Vec<T, 4> &coeff)
{
	// assert(coeff.dot(coeff) == 1);
	std::cout << "Vec constructor here" << std::endl;
	//this->coeff = Quat<T>(coeff).normalize().coeff;
	this->coeff = coeff;
}	

template <typename T>
Quat<T>::Quat(const T angle, cv::Vec<T, 3> &axis)
{
	this->coeff = {cos(angle / 2), sin(angle / 2) * axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]};
}		

template <typename T>
Quat<T>::Quat(const T qw, const T qx, const T qy, const T qz){
	this->coeff = Quat<T>(qw, qx, qy, qz).normalize().coeff;
}
*/
template <typename T>
Quat<T> log(Quat<T> &q1)
{
	return q1.log();
}

template <typename T>
Quat<T> Quat<T>::log() const
{
	cv::Vec<T, 3> v{x, y, z};
	T normV = sqrt(v.dot(v));
	T normq = norm();
	T k = normV < EPS ? 1 : acos(w / norm()) / normV;
	return Quat(log(norm()), v[0] * k, v[1] * k, v[2] *k);
}

template <typename T>
Quat<T> exp(Quat<T> &q1)
{
	return q1.exp();
}

template <typename T>
Quat<T> Quat<T>::exp() const 
{
	cv::Vec<T, 3> v{x, y, z};
	T normV = sqrt(v.dot(v));
	T k = normV < EPS ? 1 : sin(normV) / normV;
	return Quat(exp(w) * (cos(normV), v[0] * k, v[1] * k, v[2] * k));
}

template <typename T>
inline Quat<T> Quat<T>::squad(const Quat<T> &q1, const Quat<T> &q2,
							  const Quat<T> &q3, const Quat<T> &q4, T t)
{
	return slerp(slerp(q1, q4, t), slerp(q2, q3, t), 2 * t * (1 - t));
}

template <typename T>
Quat<T> Quat<T>::interPoint(const Quat<T> &q0,
									 			const Quat<T> &q1,
									 			const Quat<T> &q2) const
{
	Quat<T> qconj = q1.conjugate();
	Quat<T> c1 = log(qconj * q0);
	Quat<T> c2 = log(qconj * q2);
	Quat<T> res =  q1 * exp((- (c1 + c2) / 4)).normalize();
	return res;
}

template <typename T>
Quat<T> Quat<T>::spline(const Quat<T> &q0, const Quat<T> &q1, const Quat<T> &q2, const Quat<T> &q3, T t) const
{
	T angle;
	for (auto &i: {q0, q1, q2})
	{
		angle = sqrt(acos(q3.dot(i)));
		if (angle > CV_PI / 2)
			i = -i;
	}
	Quat<T> s1 = interPoint(q0, q1, q2);
	Quat<T> s2 = interPoint(q1, q2, q3);
	return squad(q1, s1, s2, q2, t);
}

int main(){
	cv::Vec<double, 4> v1{1,2,3,4};
	cv::Vec<double, 4> v2{5,6,7,8};
	Quat<double> q1(1,2,3,4);
	Quat<double> q2(5,6,7,8);
	double s = 10;
	std::cout << -q1 << std::endl;
	std::cout << (q1 == q2) << std::endl;
	std::cout << (q1 == q1) << std::endl;
	std::cout << q1 + q2 << std::endl;
	std::cout << q1 - q2 << std::endl;
	std::cout << q1 / s << std::endl;
	
	std::cout << q1 * q2 << std::endl;
	std::cout << v1 * v2 << std::endl;
	
	
	std::cout << q1 * s << std::endl;
	std::cout << s * q1 << std::endl;

	q1 += q2;
	std::cout << q1 << std::endl;

	q1 -= q2;

	std::cout << q1 << std::endl;

	q1 *= s;

	std::cout << q1 << std::endl;

	q1 /= s;

	std::cout << q1 << std::endl;
	q1 *= q2;


	std::cout << q1 << std::endl;
	std::cout << q1.x << std::endl;
	std::cout << q1 << std::endl;
	std::cout << q1[1]<< std::endl;



	return 0;
}
