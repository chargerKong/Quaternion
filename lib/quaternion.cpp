// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.  
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2020, Huawei Technologies Co., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and 
// limitations under the License. 
//
// Author: Longbu Wang <riskiest@gmail.com>
//         Liangqian Kong <chargerKong@126.com>
#include "quaternion.hpp"
#include <iomanip>
#include <math.h>
#define EPS 0.0001
#include <vector>
namespace cv{
template <typename T>
Quat<T>::Quat(const cv::Vec<T, 4> &coeff):w(coeff[0]), x(coeff[1]), y(coeff[2]), z(coeff[3]){}

template <typename T>
Quat<T>::Quat(const T qw, const T qx, const T qy, const T qz):w(qw), x(qx), y(qy), z(qz){}

template <typename T>
Quat<T>::Quat(const T angle, const cv::Vec<T, 3> &axis, const T qNorm)
{ 	
    T vNorm = std::sqrt(axis.dot(axis));
    if (vNorm < EPS || qNorm < EPS)
    {
        throw "this quaternion does not represent a rotation";
    }
    w = std::cos(angle / 2) * qNorm;
    x = std::sin(angle / 2) * (axis[0] / vNorm) * qNorm;
    y = std::sin(angle / 2) * (axis[1] / vNorm) * qNorm;
    z = std::sin(angle / 2) * (axis[2] / vNorm) * qNorm;
}

template <typename T>
Quat<T>::Quat(const cv::Mat &R)
{
    assert(R.rows == 3 && R.cols == 3);
    T S;
    T trace = R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2);
    if (trace > 0)
    {
        S = std::sqrt(trace + 1) * 2;
        x = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
        y = (R.at<T>(2, 0) - R.at<T>(0, 2)) / S;
        z = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
        w = 0.25 * S;
    }
    else if (R.at<T>(0, 0) > R.at<T>(1, 1) && R.at<T>(0, 0) > R.at<T>(2, 2))
    {

        S = std::sqrt(1.0 + R.at<T>(0, 0) - R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
        x = 0.25 * S;
        y = (R.at<T>(1, 0) + R.at<T>(0, 1)) / S;
        z = (R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
        w = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
    }
    else if (R.at<T>(1, 1) > R.at<T>(2, 2))
    {
        S = std::sqrt(1.0 - R.at<T>(0, 0) + R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
        x = (R.at<T>(0, 1) + R.at<T>(1, 0)) / S;
        y = 0.25 * S;
        z = (R.at<T>(1, 2) + R.at<T>(2, 1)) / S;
        w = (R.at<T>(0, 2) - R.at<T>(2, 0)) / S;
    }
    else
    {
        S = std::sqrt(1.0 - R.at<T>(0, 0) - R.at<T>(1, 1) + R.at<T>(2, 2)) * 2;
        x = (R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
        y = (R.at<T>(1, 2) + R.at<T>(2, 1)) / S;
        z = 0.25 * S;
        w = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
    }
}

template <typename T>
Quat<T>::Quat(const Vec<T, 3> &rodrigues)
{
    T tanVal = std::sqrt(rodrigues.dot(rodrigues));
    if (tanVal < EPS)
    {
        throw "This rodrigues vetor can't be transformed to a quaternion";
    }
    T angle = std::atan(tanVal);
    w = std::cos(angle);
    x = rodrigues[0] / tanVal * std::sin(angle);
    y = rodrigues[1] / tanVal * std::sin(angle);
    z = rodrigues[2] / tanVal * std::sin(angle);
}


template <typename T>
inline Quat<T> Quat<T>::operator-() const
{
    return Quat<T>(-w, -x, -y, -z);
}


template <typename T>
inline bool Quat<T>::operator==(const Quat<T> &q) const
{
    return (abs(w - q.w) < EPS && abs(x - q.x) < EPS && abs(y - q.y) < EPS && abs(z - q.z) < EPS);
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
inline Quat<T>& Quat<T>::operator/=(const Quat<T> &q1)
{
    Quat<T> q(*this * q1.inv());
    w = q.w;
    x = q.x;
    y = q.y;
    z = q.z;
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
inline Quat<T> Quat<T>::operator/(const Quat<T> &q) const
{
    return *this * q.inv();
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
            throw ("subscript exceeds the index range");
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
            throw ("subscript exceeds the index range");
    }
}

template <typename T>
std::ostream & operator<<(std::ostream &os, const Quat<T> &q)
{
    os << "Quat " << cv::Vec<T, 4>{q.w, q.x, q.y, q.z};
    return os;
}

template <typename T>
inline T Quat<T>::at(size_t index) const
{
    return (*this)[index];
}

template <typename T>
inline Quat<T> Quat<T>::conjugate() const
{
    return Quat<T>(w, -x, -y, -z);
}

template <typename T>
inline T Quat<T>::norm() const
{
    return std::sqrt(dot(*this));
}

template <typename T>
Quat<T> exp(const Quat<T> &q)
{
    return q.exp();
}

template <typename T>
Quat<T> Quat<T>::exp() const
{
    cv::Vec<T, 3> v{x, y, z};
    T normV = std::sqrt(v.dot(v));
    T k = normV < EPS ? 1 : std::sin(normV) / normV;
    return std::exp(w) * Quat<T>(std::cos(normV), v[0] * k, v[1] * k, v[2] * k);
}

template <typename T>
Quat<T> log(const Quat<T> &q, bool assumeUnit)
{
    return q.log(assumeUnit);
}

template <typename T>
Quat<T> Quat<T>::log(bool assumeUnit) const
{
    cv::Vec<T, 3> v{x, y, z};
    T vNorm = std::sqrt(v.dot(v));
    if (assumeUnit)
    {
        T k = vNorm < EPS ? 1 : std::acos(w);
        return Quat<T>(0, v[0] * k, v[1] * k, v[2] * k);
    }   
    T qNorm = norm();
    if (qNorm < EPS)
    {
        throw "This quaternion can't be applied to log";
    }
    T k = vNorm < EPS ? 1 : std::acos(w / qNorm);
    return Quat<T>(std::log(qNorm), v[0] * k, v[1] * k, v[2] *k);
}

template <typename T>
inline Quat<T> power(const Quat<T> &q1, T x, bool assumeUnit)
{
    return q1.power(x, assumeUnit);
}

template <typename T>
inline Quat<T> Quat<T>::power(T x, bool assumeUnit) const
{
    T angle = getAngle(assumeUnit);
    cv::Vec<T, 3> axis = getAxis(assumeUnit);
    if (assumeUnit)
    {
        return Quat<T>(x * angle, axis);
    }
    return std::pow(norm(), x) * Quat<T>(x * angle, axis);
}


template <typename T>
inline CV_EXPORTS_W Quat<T> sqrt(Quat<T> &q)
{
    return q.sqrt();
}

template <typename T>
inline Quat<T> Quat<T>::sqrt() const
{
    return power(0.5);
}


template <typename T>
inline Quat<T> power(const Quat<T> &p, const Quat<T> &q)
{
    return p.power(q);
}


template <typename T>
inline Quat<T> Quat<T>::power(const Quat<T> &q) const
{
    Quat<T> ans = *this * q.log();
    return ans.exp();
}

template <typename T>
inline T Quat<T>::dot(Quat<T> q1) const
{
    return w * q1.w + x * q1.x + y * q1.y + z * q1.z;
}


template <typename T>
inline Quat<T> crossProduct(const Quat<T> &p, const Quat<T> &q)
{
    return p.crossProduct(q);
}


template <typename T>
inline Quat<T> Quat<T>::crossProduct(const Quat<T> &q) const
{
    return Quat<T> (0, y * q.z - z * q.y, z * q.x - x * q.z, x * q.y - q.x * y);
}

template <typename T>
inline Quat<T> Quat<T>::normalize() const
{
    T normVal = norm();
    if (normVal < EPS)
    {
        throw "This can't be normalized.";
    }
    return Quat<T>(w / normVal, x / normVal, y / normVal, z / normVal) ;
}

template <typename T>
inline Quat<T> inv(const Quat<T> &q)
{
    return q.inv();
}


template <typename T>
inline Quat<T> Quat<T>::inv(bool assumeUnit) const
{
    if (assumeUnit)
    {
        return conjugate();
    }
    T norm2 = dot(*this);
    if (norm2 < EPS)
    {
        throw "This quaternion have not inverse";
    }
    return conjugate() / norm2;
}

template <typename T>
inline Quat<T> sinh(const Quat<T> &q)
{
    return q.sinh();
}


template <typename T>
inline Quat<T> Quat<T>::sinh() const
{
    cv::Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    // add vNorm
    T k = std::cosh(w) * std::sin(vNorm) / vNorm;
    return Quat<T>(std::sinh(w) * std::cos(vNorm), v[0] * k, v[1] * k, v[2] * k);
}


template <typename T>
inline Quat<T> cosh(const Quat<T> &q)
{
    return q.cosh();
}


template <typename T>
inline Quat<T> Quat<T>::cosh() const
{
    cv::Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = std::sinh(w) * std::sin(vNorm) / vNorm;
    return Quat<T>(std::cosh(w) * std::cos(vNorm), v[0] * k, v[1] * k, v[2] * k);
}

template <typename T>
inline Quat<T> tanh(const Quat<T> &q)
{
    return q.tanh();
}

template <typename T>
inline Quat<T> Quat<T>::tanh() const
{
    return sinh() * cosh().inv();
}


template <typename T>
inline Quat<T> sin(const Quat<T> &q)
{
    return q.sin();
}


template <typename T>
inline Quat<T> Quat<T>::sin() const
{
    cv::Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = std::cos(w) * std::sinh(vNorm) / vNorm;
    return Quat<T>(std::sin(w) * std::cosh(vNorm), v[0] * k, v[1] * k, v[2] * k);
}

template <typename T>
inline Quat<T> cos(const Quat<T> &q)
{
    return q.cos();
}

template <typename T>
inline Quat<T> Quat<T>::cos() const
{
    cv::Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = std::sin(w) * std::sinh(vNorm) / vNorm;
    return Quat<T>(std::cos(w) * std::cosh(vNorm), -v[0] * k, -v[1] * k, -v[2] * k);
}

template <typename T>
inline Quat<T> tan(const Quat<T> &q)
{
    return q.tan();
}

template <typename T>
inline Quat<T> Quat<T>::tan() const
{
    return sin() * cos().inv();
}

template <typename T>
inline Quat<T> asinh(Quat<T> &q)
{
    return q.asinh();
}

template <typename T>
inline Quat<T> Quat<T>::asinh() const
{
    Quat<T> c1 = *this * *this + Quat<T>(1,0,0,0);
    Quat<T> c2 = c1.power(0.5) + *this;
    return c2.log();
    // return log(*this + power(*this * *this + Quat<T>(1,0,0,0), 0.5));
}

template <typename T>
inline Quat<T> acosh(const Quat<T> &q)
{
    return q.acosh();
}

template <typename T>
inline Quat<T> Quat<T>::acosh() const
{
    Quat<T> c1 = *this * *this - Quat<T>(1,0,0,0);
    Quat<T> c2 = c1.power(0.5) + *this;
    return c2.log();
    //return cv::log(*this + cv::power(*this * *this - Quat<T>(1,0,0,0), 0.5));
}

template <typename T>
inline Quat<T> atanh(const Quat<T> &q)
{
    return q.atanh();
}

template <typename T>
inline Quat<T> Quat<T>::atanh() const
{
    Quat<T> ident(1, 0, 0, 0);
    Quat<T> c1 = (ident + *this).log();
    Quat<T> c2 = (ident - *this).log();
    return 1 / 2 * (c1 - c2);
    //return 1/2 * (cv::log(ident + *this) - cv::log(ident - *this));
}

template <typename T>
inline Quat<T> asin(const Quat<T> &q)
{
    return q.asin();
}

template <typename T>
inline Quat<T> Quat<T>::asin() const
{
    Quat<T> v(0, x, y, z);
    T vNorm = v.norm();
    return -v / vNorm * (*this * v / vNorm).asinh();
}

template <typename T>
inline Quat<T> acos(const Quat<T> &q)
{
    return q.acos();
}

template <typename T>
inline Quat<T> Quat<T>::acos() const
{
    Quat<T> v(0, x, y, z);
    T vNorm = v.norm();
    return -v / vNorm * acosh();
}

template <typename T>
inline Quat<T> atan(const Quat<T> &q)
{
    return q.atan();
}

template <typename T>
inline Quat<T> Quat<T>::atan() const
{
    Quat<T> v(0, x, y, z);
    T vNorm = v.norm();
    return -v / vNorm * (*this * v / vNorm).atanh();
}

template <typename T>
inline T Quat<T>::getAngle(bool assumeUnit) const
{
    if (assumeUnit)
    {
        return 2 * std::acos(w);
    }
    if (x * x + y * y + z * z < EPS || norm() < EPS )
    {
        throw "this quaternion does not represent a rotation";
    }
    return 2 * std::acos(w / norm());
}

template <typename T>
inline Vec<T, 3> Quat<T>::getAxis(bool assumeUnit) const
{
    T angle = getAngle(assumeUnit);
    if (assumeUnit)
    {
        return Vec<T, 3>{x, y, z} / std::sin(angle / 2);
    }
    return Vec<T, 3> {x, y, z} / (norm() * std::sin(angle / 2));
}

template <typename T>
cv::Mat Quat<T>::toRotMat4x4() const
{
    T dotVal = dot(*this);
    cv::Matx<T, 4, 4> R{
         2* w * w - dotVal, -2 * w * x        , -2 * w * y         ,  - 2 * w * z,
         -2 * w * x       , dotVal - 2 * x * x, - 2 * x * y        , -2 * x * z,
         2 * w * y        , -2 * x * y        , dotVal - 2 * y * y , -2 * z * y,
         2 * w * z        , -2 * x * z        , -2 * y * z         , dotVal - 2 * z * z};
    return cv::Mat(R.t()).t();
}

template <typename T>
cv::Mat Quat<T>::toRotMat3x3(bool assumeUnit) const
{
    T a = w, b = x, c = y, d = z;
    if (!assumeUnit)
    {
        Quat<T> qTemp = normalize();
        a = qTemp.w;
        b = qTemp.x;
        c = qTemp.y;
        d = qTemp.z;
    }
    cv::Matx<T, 3, 3> R{
          1 - 2 * (c * c + d * d), 2 * (b * c + a * d)    , 2 * (b * d - a * c),
          2 * (b * c - a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d + a * b),
          2 * (b * d + a * c)    , 2 * (c * d - a * b)    , 1 - 2 * (b * b + c * c)};
    return cv::Mat(R).t();
}

template <typename T>
Vec<T, 3> Quat<T>::toRodrigues() const
{
    if (abs(w) < EPS)
    {
        throw "the rotate vector is indeterminte";
    }
    return Vec<T, 3>{x / w, y / w, z / w};
}

template <typename T>
cv::Vec<T, 4> Quat<T>::toVec() const
{
    return cv::Vec<T, 4>{w, x, y, z};
}

template <typename T>
Quat<T> Quat<T>::lerp(const Quat<T> &q0, const Quat<T> &q1, const T t)
{
    return (1 - t) * q0 + t * q1;
}

template <typename T>
Quat<T> Quat<T>::slerp(Quat<T> &q0, Quat<T> &q1, const T t, bool assumeUnit)
{
    Quatd v0(q0);
    Quatd v1(q1);

    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
        // add warning:
    }
    T cosTheta = v0.dot(v1);
    T DOT_THRESHOLD = 0.995;
    if (cosTheta > DOT_THRESHOLD)
    {
        return nlerp(v0, v1, t, true);
    }
    /*
    if (cosTheta < 0)
    {
        v0 = -v0;
        cosTheta = -cosTheta;
    }
    */
    T sinTheta = std::sqrt(1 - cosTheta * cosTheta);
    T angle = atan2(sinTheta, cosTheta);
    return (std::sin((1 - t) * angle) / (sinTheta) * v0 + std::sin(t * angle) / (sinTheta) * v1).normalize();
}


template <typename T>
inline Quat<T> Quat<T>::nlerp(Quat<T> &q0, Quat<T> &q1, const T t, bool assumeUnit)
{
    Quat<T> q2 = q0;
    if (q1.dot(q0) < 0)
    {
        q2 = -q2;
    }
    if (assumeUnit)
    {
        return ((1 - t) * q2 + t * q1).normalize();
    }
    // add warning
    q0 = q0.normalize();
    q1 = q1.normalize();
    return ((1 - t) * q2 + t * q1).normalize();
}


template <typename T>
inline bool Quat<T>::isNormal() const
{

    double normVar = norm();
    if ((normVar > 1 - EPS) && (normVar < 1 + EPS))
        return true;
    return false;
}

template <typename T>
inline void Quat<T>::assertNormal() const
{
    if (!isNormal())
        throw ("Quaternions should be normalized");
}

template <typename T>
inline Quat<T> Quat<T>::squad(Quat<T> &q0, Quat<T> &q1,
							  Quat<T> &q2, Quat<T> &q3, const T t, bool assumeUnit)
{
    if (!assumeUnit)
    {
        q0 = q0.normalize();
        q1 = q1.normalize();
        q2 = q2.normalize();
        q3 = q3.normalize();
        // add warning in inter
    }

    Quat<T> c0 = slerp(q0, q3, t, assumeUnit);
    Quat<T> c1 = slerp(q1, q2, t, assumeUnit);
    return slerp(c0, c1, 2 * t * (1 - t), assumeUnit);
}

template <typename T>
Quat<T> Quat<T>::interPoint(Quat<T> &q0, Quat<T> &q1,
							Quat<T> &q2, bool assumeUnit)
{
    if (!assumeUnit)
    {
        q0 = q0.normalize();
        q1 = q1.normalize();
        q2 = q2.normalize();
        // add warning in inter
    }
    return q1 * cv::exp(-(cv::log(q1.conjugate() * q0) + (cv::log(q1.conjugate() * q2))) / 4);
}

template <typename T>
Quat<T> Quat<T>::spline(Quat<T> &q0, Quat<T> &q1, Quat<T> &q2, Quat<T> &q3, const T t, bool assumeUnit)
{
    Quatd v0, v1, v2, v3;
    v0 = q0;
    v1 = q1;
    v2 = q2;
    v3 = q3;
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
        v2 = v2.normalize();
        v3 = v3.normalize();
        // add warning
    }
    T cosTheta;
    std::vector<Quat<T>> vec{v0, v1, v2, v3};
    //std::cout << vec << std::endl;
    for (size_t i = 0; i < 3; ++i)
    {

        cosTheta = vec[i].dot(vec[i + 1]);
        if (cosTheta < 0)
        {
            vec[i] = -vec[i];
        }
    }
    //std::cout << vec << std::endl;

    Quat<T> s1 = interPoint(v0, v1, v2, true);
    Quat<T> s2 = interPoint(v1, v2, v3, true);
    
    return squad(v1, s1, s2, v2, t, assumeUnit);
}

/*
static Quat<T> splinet(std::vector<Quat<T>> vec, const T t, bool assumeUnit,std::string method)
{
    for (auto beg = vec.begin(), end = vec.end();beg != end; ++beg
    {

    }
}
*/

}
/*a
int main(){
	double angle = (CV_PI / 2);
	cv::Vec<double, 3> axis{1/sqrt(3), 1/sqrt(3), 1/sqrt(3)};
	
	Quat<double> tr1(0, axis);
	Quat<double> tr2(angle / 2, axis);
	Quat<double> tr3(angle, cv::Vec<double,3>{1/sqrt(3),1/sqrt(3),1/sqrt(3)});
	Quat<double> tr4(angle, cv::Vec<double,3>{1/sqrt(2),0,1/(sqrt(2))});
	Quat<double> tr5(angle, cv::Vec<double,3>{2/sqrt(5),0/sqrt(5),-1/(sqrt(5))});
		
	cv::Mat p = (cv::Mat_<double>(3,1) << 1,0,0);
	cv::Mat ans;
	Quat<double> t_12;
	std::vector<Quat<double>> trs{tr1,tr2,tr3,tr4,tr5};
	try
	{	
	for (size_t i = 0; i < 4; ++i)
	{
		for (auto &j:{0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0})
		{
			t_12 = Quat<double>::slerp(trs[i],trs[i+1],j);
			ans = (t_12.toRotMat3x3() * p).t();
			std::cout << ans << std::endl;
		}
	}
	std::cout << "here"<<trs[5] << std::endl;
	std::cout << "\n" << std::endl;

		for (auto &j:{0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0})
		{
	t_12 = Quat<double>::spline(trs[0],trs[0],trs[1],trs[2],j,true);
	ans = (t_12.toRotMat3x3() * p).t();
	std::cout << ans << std::endl;
		}
	for (size_t i = 0; i < 5 - 2; ++i)
	{
		for (auto &j:{0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0})
		{
			//t_12 = Quat<double>::slerp(trs[i],trs[i+1],j);
			Quat<double> q0, q1, q2, q3;
			q0 = trs[i];
			q1 = trs[i + 1];
			q2 = trs[i + 2];
			q3 = trs[i + 3];
				if(i == 2){
					q3 = trs[i+2];
				}

			t_12 = Quat<double>::spline(q0,q1,q2,q3,j,true);
			ans = (t_12.toRotMat3x3() * p).t();
			std::cout << ans << std::endl;
		}
	}	
//#pragma message("abc");
		
	}
	catch(const char* msg)
	{
			std::cerr << msg << std::endl;
	}
	std::cout << "\ninterpoint\n";
	Quat<double> q(0,1,0,0);
	
		t_12 = Quat<double>::slerp(tr4,tr5,1);
		//
		//t_12 = Quat<double>::spline(tr3,tr4,tr5,tr5,1,true);
		ans = (t_12.toRotMat3x3() * p).t();
		std::cout << ans << std::endl;
	
	std::cout << "test here\n";
	Quat<double> t1(0,1,0,0);
	std::cout << tr4.toRotMat3x3() * p << std::endl;
	std::cout << (-tr4).toRotMat3x3() * p << std::endl;
	
	//std::cout << asin(q) << std::endl;
	//std::cout << q.sinh() << std::endl;


	


	return 0;
}*/
