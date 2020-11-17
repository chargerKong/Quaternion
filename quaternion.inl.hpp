/// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2020, Huawei Technologies Co., Ltd. All rights reserved.
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
// Author: Liangqian Kong <chargerKong@126.com>
//         Longbu Wang <riskiest@gmail.com>

#ifndef OPENCV_CORE_QUATERNIONINL_HPP
#define OPENCV_CORE_QUATERNIONINL_HPP

#include "quaternion.hpp"
//@cond IGNORE
///////////////////////////////////////////////////////////////////////////////////////
//Implementation
namespace cv{
template <typename T>
Quat<T>::Quat(const Vec<T, 4> &coeff):w(coeff[0]), x(coeff[1]), y(coeff[2]), z(coeff[3]){}

template <typename T>
Quat<T>::Quat(const T qw, const T qx, const T qy, const T qz):w(qw), x(qx), y(qy), z(qz){}

template <typename T>
Quat<T> Quat<T>::createFromAngleAxis(const T angle, const Vec<T, 3> &axis)
{
    T w, x, y, z;
    T vNorm = std::sqrt(axis.dot(axis));
    if (vNorm < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "this quaternion does not represent a rotation");
    }
    w = std::cos(angle / 2);
    x = std::sin(angle / 2) * (axis[0] / vNorm);
    y = std::sin(angle / 2) * (axis[1] / vNorm);
    z = std::sin(angle / 2) * (axis[2] / vNorm);
    return Quat<T>(w, x, y, z);
}

template <typename T>
Quat<T> Quat<T>::createFromRotMat(InputArray _R)
{
    Mat R = _R.getMat();
    if (R.rows != 3 || R.cols != 3)
    {
        CV_Error(Error::StsBadArg, "Cannot convert matrix to quaternion: rotation matrix should be a 3x3 matrix");
    }
    T S, w, x, y, z;
    T trace = R.at<T>(0, 0) + R.at<T>(1, 1) + R.at<T>(2, 2);
    if (trace > 0)
    {
        S = std::sqrt(trace + 1) * 2;
        x = (R.at<T>(1, 2) - R.at<T>(2, 1)) / S;
        y = (R.at<T>(2, 0) - R.at<T>(0, 2)) / S;
        z = (R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
        w = -0.25 * S;
    }
    else if (R.at<T>(0, 0) > R.at<T>(1, 1) && R.at<T>(0, 0) > R.at<T>(2, 2))
    {

        S = std::sqrt(1.0 + R.at<T>(0, 0) - R.at<T>(1, 1) - R.at<T>(2, 2)) * 2;
        x = -0.25 * S;
        y = -(R.at<T>(1, 0) + R.at<T>(0, 1)) / S;
        z = -(R.at<T>(0, 2) + R.at<T>(2, 0)) / S;
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
        w = -(R.at<T>(0, 1) - R.at<T>(1, 0)) / S;
    }
    return Quat<T> (w, x, y, z);
}

template <typename T>
Quat<T> Quat<T>::createFromRvec(InputArray _rvec)
{
    Mat rmat = _rvec.getMat();
    if (!((rmat.cols == 1 && rmat.rows == 3) || (rmat.cols == 3 && rmat.rows == 1))){
        CV_Error(Error::StsBadArg, "Cannot convert rotation vector to quaternion: The length of rotation vector should be 3");
    }
    Vec<T, 3> rvec(rmat);
    T psi = std::sqrt(rvec.dot(rvec));
    if (abs(psi) < CV_QUAT_EPS){
        return Quat<T> (1, 0, 0, 0);
    }
    Vec<T, 3> axis = rvec / psi;
    return createFromAngleAxis(psi, axis);
}

template <typename T>
inline Quat<T> Quat<T>::operator-() const
{
    return Quat<T>(-w, -x, -y, -z);
}


template <typename T>
inline bool Quat<T>::operator==(const Quat<T> &q) const
{
    return (abs(w - q.w) < CV_QUAT_EPS && abs(x - q.x) < CV_QUAT_EPS && abs(y - q.y) < CV_QUAT_EPS && abs(z - q.z) < CV_QUAT_EPS);
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
    Vec<T, 4> q{w, x, y, z};
    Vec<T, 4> q2{q1.w, q1.x, q1.y, q1.z};
    return Quat<T>(q * q2);
}


template <typename T, typename S>
Quat<T> operator*(const Quat<T> &q1, const S a)
{
    return Quat<T>(a * q1.w, a * q1.x, a * q1.y, a * q1.z);
}

template <typename T, typename S>
Quat<T> operator*(const S a, const Quat<T> &q1)
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
        case 1:
            return x;
        case 2:
            return y;
        case 3:
            return z;
        default:
            CV_Error(Error::StsOutOfRange, "subscript exceeds the index range");
    }
}

template <typename T>
inline T& Quat<T>::operator[](std::size_t n)
{
    switch (n) {
        case 0:
            return w;
        case 1:
            return x;
        case 2:
            return y;
        case 3:
            return z;
        default:
            CV_Error(Error::StsOutOfRange, "subscript exceeds the index range");
    }
}

template <typename T>
std::ostream & operator<<(std::ostream &os, const Quat<T> &q)
{
    os << "Quat " << Vec<T, 4>{q.w, q.x, q.y, q.z};
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
    Vec<T, 3> v{x, y, z};
    T normV = std::sqrt(v.dot(v));
    T k = normV < CV_QUAT_EPS ? 1 : std::sin(normV) / normV;
    return std::exp(w) * Quat<T>(std::cos(normV), v[0] * k, v[1] * k, v[2] * k);
}

template <typename T>
Quat<T> log(const Quat<T> &q, AssumeType assumeUnit)
{
    return q.log(assumeUnit);
}

template <typename T>
Quat<T> Quat<T>::log(AssumeType assumeUnit) const
{
    Vec<T, 3> v{x, y, z};
    T vNorm = std::sqrt(v.dot(v));
    if (assumeUnit)
    {
        T k = vNorm < CV_QUAT_EPS ? 1 : std::acos(w) / vNorm;
        return Quat<T>(0, v[0] * k, v[1] * k, v[2] * k);
    }
    T qNorm = norm();
    if (qNorm < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "Cannot apply this quaternion to log function: undefined");
    }
    T k = vNorm < CV_QUAT_EPS ? 1 : std::acos(w / qNorm) / vNorm;
    return Quat<T>(std::log(qNorm), v[0] * k, v[1] * k, v[2] *k);
}

template <typename T, typename _T>
inline Quat<T> power(const Quat<T> &q1, _T alpha, AssumeType assumeUnit)
{
    return q1.power(alpha, assumeUnit);
}

template <typename T>
template <typename _T>
inline Quat<T> Quat<T>::power(_T alpha, AssumeType assumeUnit) const
{
    if (x * x + y * y + z * z > CV_QUAT_EPS)
    {
        T angle = getAngle(assumeUnit);
        Vec<T, 3> axis = getAxis(assumeUnit);
        if (assumeUnit)
        {
            return createFromAngleAxis(alpha * angle, axis);
        }
        return std::pow(norm(), alpha) * createFromAngleAxis(alpha * angle, axis);
    }
    else
    {
        return std::pow(norm(), alpha) * Quat<T>(w, x, y, z);
    }
}


template <typename T>
inline Quat<T> sqrt(const Quat<T> &q, AssumeType assumeUnit)
{
    return q.sqrt(assumeUnit);
}

template <typename T>
inline Quat<T> Quat<T>::sqrt(AssumeType assumeUnit) const
{
    return power(0.5, assumeUnit);
}


template <typename T>
inline Quat<T> power(const Quat<T> &p, const Quat<T> &q, AssumeType assumeUnit)
{
    return p.power(q, assumeUnit);
}


template <typename T>
inline Quat<T> Quat<T>::power(const Quat<T> &q, AssumeType assumeUnit) const
{
    return cv::exp(q * log(assumeUnit));
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
    if (normVal < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "Cannot normalize this quaternion: the norm is too small.");
    }
    return Quat<T>(w / normVal, x / normVal, y / normVal, z / normVal) ;
}

template <typename T>
inline Quat<T> inv(const Quat<T> &q, AssumeType assumeUnit)
{
    return q.inv(assumeUnit);
}


template <typename T>
inline Quat<T> Quat<T>::inv(AssumeType assumeUnit) const
{
    if (assumeUnit)
    {
        return conjugate();
    }
    T norm2 = dot(*this);
    if (norm2 < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "This quaternion do not have inverse quaternion");
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
    Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = vNorm < CV_QUAT_EPS ? 1 : std::cosh(w) * std::sin(vNorm) / vNorm;
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
    Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = vNorm < CV_QUAT_EPS ? 1 : std::sinh(w) * std::sin(vNorm) / vNorm;
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
    Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = vNorm < CV_QUAT_EPS ? 1 : std::cos(w) * std::sinh(vNorm) / vNorm;
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
    Vec<T, 3> v{x, y ,z};
    T vNorm = std::sqrt(v.dot(v));
    T k = vNorm < CV_QUAT_EPS ? 1 : std::sin(w) * std::sinh(vNorm) / vNorm;
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
inline Quat<T> asinh(const Quat<T> &q)
{
    return q.asinh();
}

template <typename T>
inline Quat<T> Quat<T>::asinh() const
{
    return cv::log(*this + cv::power(*this * *this + Quat<T>(1, 0, 0, 0), 0.5));
}

template <typename T>
inline Quat<T> acosh(const Quat<T> &q)
{
    return q.acosh();
}

template <typename T>
inline Quat<T> Quat<T>::acosh() const
{
    return cv::log(*this + cv::power(*this * *this - Quat<T>(1,0,0,0), 0.5));
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
    return 0.5 * (c1 - c2);
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
    T k = vNorm < CV_QUAT_EPS ? 1 : vNorm;
    return -v / k * (*this * v / k).asinh();
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
    T k = vNorm < CV_QUAT_EPS ? 1 : vNorm;
    return -v / k * acosh();
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
    T k = vNorm < CV_QUAT_EPS ? 1 : vNorm;
    return -v / k * (*this * v / k).atanh();
}

template <typename T>
inline T Quat<T>::getAngle(AssumeType assumeUnit) const
{
    if (assumeUnit)
    {
        return 2 * std::acos(w);
    }
    if (norm() < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "This quaternion does not represent a rotation");
    }
    return 2 * std::acos(w / norm());
}

template <typename T>
inline Vec<T, 3> Quat<T>::getAxis(AssumeType assumeUnit) const
{
    T angle = getAngle(assumeUnit);
    if (assumeUnit)
    {
        return Vec<T, 3>{x, y, z} / std::sin(angle / 2);
    }
    return Vec<T, 3> {x, y, z} / (norm() * std::sin(angle / 2));
}

template <typename T>
Mat Quat<T>::toRotMat4x4(AssumeType assumeUnit) const
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
    Matx<T, 4, 4> R{
        1 - 2 * (c * c + d * d), 2 * (b * c - a * d)    , 2 * (b * d + a * c)    , 0,
        2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * b)    , 0,
        2 * (b * d - a * c)    , 2 * (c * d + a * b)    , 1 - 2 * (b * b + c * c), 0,
        0                      , 0                      , 0                      , 1,
    };
    return Mat(R);
}

template <typename T>
Mat Quat<T>::toRotMat3x3(AssumeType assumeUnit) const
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
    Matx<T, 3, 3> R{
        1 - 2 * (c * c + d * d), 2 * (b * c - a * d)    , 2 * (b * d + a * c),
        2 * (b * c + a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d - a * b),
        2 * (b * d - a * c)    , 2 * (c * d + a * b)    , 1 - 2 * (b * b + c * c)
    };
    return Mat(R);
}

template <typename T>
Vec<T, 3> Quat<T>::toRotVec(AssumeType assumeUnit) const
{
    T angle = getAngle(assumeUnit);
    Vec<T, 3> axis = getAxis(assumeUnit);
    return angle * axis;
}

template <typename T>
Vec<T, 4> Quat<T>::toVec() const
{
    return Vec<T, 4>{w, x, y, z};
}

template <typename T>
Quat<T> Quat<T>::lerp(const Quat<T> &q0, const Quat<T> &q1, const T t)
{
    return (1 - t) * q0 + t * q1;
}

template <typename T>
Quat<T> Quat<T>::slerp(const Quat<T> &q0, const Quat<T> &q1, const T t, AssumeType assumeUnit, bool directChange)
{
    Quatd v0(q0);
    Quatd v1(q1);
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
    }
    T cosTheta = v0.dot(v1);
    constexpr T DOT_THRESHOLD = 0.995;
    if (cosTheta > DOT_THRESHOLD)
    {
        return nlerp(v0, v1, t, ASSUME_UNIT);
    }

    if (directChange && cosTheta < 0)
    {
        v0 = -v0;
        cosTheta = -cosTheta;
    }
    T sinTheta = std::sqrt(1 - cosTheta * cosTheta);
    T angle = atan2(sinTheta, cosTheta);
    return (std::sin((1 - t) * angle) / (sinTheta) * v0 + std::sin(t * angle) / (sinTheta) * v1).normalize();
}


template <typename T>
inline Quat<T> Quat<T>::nlerp(const Quat<T> &q0, const Quat<T> &q1, const T t, AssumeType assumeUnit)
{
    Quat<T> v0(q0), v1(q1);
    if (v1.dot(v0) < 0)
    {
        v0 = -v0;
    }
    if (assumeUnit)
    {
        return ((1 - t) * v0 + t * v1).normalize();
    }
    v0 = v0.normalize();
    v1 = v1.normalize();
    return ((1 - t) * v0 + t * v1).normalize();
}


template <typename T>
inline bool Quat<T>::isNormal(T eps) const
{

    double normVar = norm();
    if ((normVar > 1 - eps) && (normVar < 1 + eps))
        return true;
    return false;
}

template <typename T>
inline void Quat<T>::assertNormal(T eps) const
{
    if (!isNormal(eps))
        CV_Error(Error::StsBadArg, "Quaternion should be normalized");
}


template <typename T>
inline Quat<T> Quat<T>::squad(const Quat<T> &q0, const Quat<T> &q1,
                            const Quat<T> &q2, const Quat<T> &q3,
                            const T t, AssumeType assumeUnit,
                            bool directChange)
{
    Quat<T> v0(q0), v1(q1), v2(q2), v3(q3);
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
        v2 = v2.normalize();
        v3 = v3.normalize();
    }

    Quat<T> c0 = slerp(v0, v3, t, assumeUnit, directChange);
    Quat<T> c1 = slerp(v1, v2, t, assumeUnit, directChange);
    return slerp(c0, c1, 2 * t * (1 - t), assumeUnit, directChange);
}

template <typename T>
Quat<T> Quat<T>::interPoint(const Quat<T> &q0, const Quat<T> &q1,
                            const Quat<T> &q2, AssumeType assumeUnit)
{
    Quat<T> v0(q0), v1(q1), v2(q2);
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
        v2 = v2.normalize();
    }
    return v1 * cv::exp(-(cv::log(v1.conjugate() * v0, assumeUnit) + (cv::log(v1.conjugate() * v2, assumeUnit))) / 4);
}

template <typename T>
Quat<T> Quat<T>::spline(const Quat<T> &q0, const Quat<T> &q1, const Quat<T> &q2, const Quat<T> &q3, const T t, AssumeType assumeUnit)
{
    Quatd v0(q0), v1(q1), v2(q2), v3(q3);
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
        v2 = v2.normalize();
        v3 = v3.normalize();
    }
    T cosTheta;
    std::vector<Quat<T>> vec{v0, v1, v2, v3};
    for (size_t i = 0; i < 3; ++i)
    {
        cosTheta = vec[i].dot(vec[i + 1]);
        if (cosTheta < 0)
        {
            vec[i + 1] = -vec[i + 1];
        }
    }
    Quat<T> s1 = interPoint(vec[0], vec[1], vec[2], ASSUME_UNIT);
    Quat<T> s2 = interPoint(vec[1], vec[2], vec[3], ASSUME_UNIT);
    return squad(vec[1], s1, s2, vec[2], t, assumeUnit, ASSUME_NOT_UNIT);
}

template <typename T>
DualQuat<T>::DualQuat(T w, T x, T y, T z, T w_, T x_, T y_, T z_):w(w), x(x), y(y), z(z),
                                                                  w_(w_), x_(x_), y_(y_), z_(z_){};

template <typename T>
DualQuat<T> DualQuat<T>::createFromQuat(const Quat<T> &realPart, const Quat<T> &dualPart)
{
    T w = realPart.w;
    T x = realPart.x;
    T y = realPart.y;
    T z = realPart.z;
    T w_ = dualPart.w;
    T x_ = dualPart.x;
    T y_ = dualPart.y;
    T z_ = dualPart.z;
    return DualQuat<T>(w, x, y, z, w_, x_, y_, z_);
}

template <typename T>
DualQuat<T> DualQuat<T>::createFromAngleAxisTrans(T angle, const Vec<T, 3> &axis, const Quat<T> &trans)
{
    Quat<T> r = Quat<T>::createFromAngleAxis(angle, axis);
    return createFromQuat(r, trans * r / 2);
}

template <typename T>
inline bool DualQuat<T>::operator==(const DualQuat<T> &q) const
{
    return (abs(w - q.w) < CV_QUAT_EPS && abs(x - q.x) < CV_QUAT_EPS &&
            abs(y - q.y) < CV_QUAT_EPS && abs(z - q.z) < CV_QUAT_EPS &&
            abs(w_ - q.w_) < CV_QUAT_EPS && abs(x_ - q.x_) < CV_QUAT_EPS &&
            abs(y_ - q.y_) < CV_QUAT_EPS && abs(z_ - q.z_) < CV_QUAT_EPS);
}

template <typename T>
inline Quat<T> DualQuat<T>::getRealQuat() const
{
    return Quat<T>(w, x, y, z);
}

template <typename T>
inline Quat<T> DualQuat<T>::getDualQuat() const
{
    return Quat<T>(w_, x_, y_, z_);
}

template <typename T>
inline Quat<T> DualQuat<T>::getRotation(AssumeType assumeUnit) const
{
    if (assumeUnit)
    {
        return getRealQuat();
    }
    return getRealQuat().normalize();
}

template <typename T>
Quat<T> DualQuat<T>::getTranslation(AssumeType assumeUnit) const
{
    if (assumeUnit)
    {
        return 2 * getDualQuat() * getRealQuat().conjugate();
    }
    // TODO
    return 2 * getDualQuat() * getRealQuat().conjugate();
}

template<typename T>
inline DualQuat<T> DualQuat<T>::conjugate() const
{
    return DualQuat<T>(w, -x, -y, -z, w_, -x_, -y_, -z_);
}

template <typename T>
inline T DualQuat<T>::norm() const
{
    return std::sqrt(w * w + x * x + y * y + z * z + w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
}

template <typename T>
inline DualQuat<T> DualQuat<T>::normalize() const
{
    T rpNorm = getRealQuat().norm();
    if (rpNorm < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "Cannot normalize this dual quaternion: the norm is too small.");
    }
    Quat<T> dualpart = getDualQuat() / rpNorm;

    return *this / rpNorm;
}

template <typename T>
inline T DualQuat<T>::dot(DualQuat<T> q) const
{
    return q.w * w + q.x * x + q.y * y + q.z * z + q.w_ * w_ + q.x_ * x_ + q.y_ * y_ + q.z_ * z_;
}

template <typename T>
inline DualQuat<T> DualQuat<T>::inv(AssumeType assumeUnit) const
{
    Quat<T> p = getRealQuat();
    Quat<T> q = getDualQuat();
    if (assumeUnit)
    {
        //return conjugate();
        return DualQuat<T>::createFromQuat(p.inv(), -p.inv() * q * p.inv());
    }
    // TODO, have problem
    T norm2 = dot(*this);
    if (norm2 < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "This dual quaternion do not have inverse dual quaternion");
    }
    //return conjugate() / norm2;
    return DualQuat<T>::createFromQuat(p.inv(), -p.inv() * q * p.inv());
  }


template <typename T>
inline DualQuat<T> DualQuat<T>::operator-(const DualQuat<T> &q) const
{
    return DualQuat<T>(w - q.w, x - q.x, y - q.y, z - q.z, w_ - q.w_, x_ - q.x_, y_ - q.y_, z_ - q.z_);
}

template <typename T>
inline DualQuat<T> DualQuat<T>::operator-() const
{
    return DualQuat<T>(-w, -x, -y, -z, -w_, -x_, -y_, -z_);
}

template <typename T>
inline DualQuat<T> DualQuat<T>::operator+(const DualQuat<T> &q) const
{
    return DualQuat<T>(w + q.w, x + q.x, y + q.y, z + q.z, w_ + q.w_, x_ + q.x_, y_ + q.y_, z_ + q.z_);
}

template <typename T>
inline DualQuat<T> DualQuat<T>::operator*(const DualQuat<T> &q) const
{
    Quat<T> A = getRealQuat();
    Quat<T> B = getDualQuat();
    Quat<T> C = q.getRealQuat();
    Quat<T> D = q.getDualQuat();
    return DualQuat<T>::createFromQuat(A * C, A * D + B * C);
}

template <typename T>
inline DualQuat<T> DualQuat<T>::operator/(T a) const
{
    return DualQuat<T>(w / a, x / a, y / a, z / a, w_ / a, x_ / a, y_ / a, z_ / a);
}

template <typename T>
std::ostream & operator<<(std::ostream &os, const DualQuat<T> &q)
{
    os << "DualQuat " << Vec<T, 8>{q.w, q.x, q.y, q.z, q.w_, q.x_, q.y_, q.z_};
    return os;
}

}//namepsace

//! @endcond

#endif /*OPENCV_CORE_QUATERNIONINL_HPP*/
