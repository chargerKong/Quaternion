// This file is part of OpenCV project.
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

#include "quaternion.hpp"
#include <vector>

namespace cv{

template <typename T>
Quat<T>::Quat(const Vec<T, 4> &coeff):w(coeff[0]), x(coeff[1]), y(coeff[2]), z(coeff[3]){}

template <typename T>
Quat<T>::Quat(const T qw, const T qx, const T qy, const T qz):w(qw), x(qx), y(qy), z(qz){}

template <typename T>
Quat<T>::Quat(const T angle, const cv::Vec<T, 3> &axis, const T qNorm)
{
    T vNorm = std::sqrt(axis.dot(axis));
    if (vNorm < CV_QUAT_EPS || qNorm < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "this quaternion does not represent a rotation");
    }
    w = std::cos(angle / 2) * qNorm;
    x = std::sin(angle / 2) * (axis[0] / vNorm) * qNorm;
    y = std::sin(angle / 2) * (axis[1] / vNorm) * qNorm;
    z = std::sin(angle / 2) * (axis[2] / vNorm) * qNorm;
}

template <typename T>
Quat<T>::Quat(const cv::Mat &R)
{
    if (R.rows != 3 || R.cols != 3)
    {
        CV_Error(Error::StsBadArg, "Cannot matrix to quaternion: rotation matrix should be a 3x3 matrix");
    }
    T S;
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
}

template <typename T>
Quat<T>::Quat(const Vec<T, 3> &rvec)
{
    T tanVal = std::sqrt(rvec.dot(rvec));
    if (tanVal < CV_QUAT_EPS)
    {
        w = 0;
        x = 0;
        y = 0;
        z = 0;
    }
    else
    {
        T angle = std::atan(tanVal);
        w = std::cos(angle);
        x = rvec[0] / tanVal * std::sin(angle);
        y = rvec[1] / tanVal * std::sin(angle);
        z = rvec[2] / tanVal * std::sin(angle);
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
            CV_Error(Error::StsOutOfRange, "subscript exceeds the index range");
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
            CV_Error(Error::StsOutOfRange, "subscript exceeds the index range");
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
    T k = normV < CV_QUAT_EPS ? 1 : std::sin(normV) / normV;
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
inline Quat<T> power(const Quat<T> &q1, _T alpha, bool assumeUnit)
{
    return q1.power(alpha, assumeUnit);
}

template <typename T>
template <typename _T>
inline Quat<T> Quat<T>::power(_T alpha, bool assumeUnit) const
{
    if (x * x + y * y + z * z > CV_QUAT_EPS)
    {
        T angle = getAngle(assumeUnit);
        cv::Vec<T, 3> axis = getAxis(assumeUnit);
        if (assumeUnit)
        {
            return Quat<T>(alpha * angle, axis);
        }
        return std::pow(norm(), alpha) * Quat<T>(alpha * angle, axis);
    }
    else
    {
        return std::pow(norm(), alpha) * Quat<T>(w, x, y, z);
    }
}


template <typename T>
inline Quat<T> sqrt(const Quat<T> &q, bool assumeUnit)
{
    return q.sqrt(assumeUnit);
}

template <typename T>
inline Quat<T> Quat<T>::sqrt(bool assumeUnit) const
{
    return power(0.5, assumeUnit);
}


template <typename T>
inline Quat<T> power(const Quat<T> &p, const Quat<T> &q, bool assumeUnit)
{
    return p.power(q, assumeUnit);
}


template <typename T>
inline Quat<T> Quat<T>::power(const Quat<T> &q, bool assumeUnit) const
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
inline Quat<T> inv(const Quat<T> &q, bool assumeUnit)
{
    return q.inv(assumeUnit);
}


template <typename T>
inline Quat<T> Quat<T>::inv(bool assumeUnit) const
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
    cv::Vec<T, 3> v{x, y ,z};
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
    cv::Vec<T, 3> v{x, y ,z};
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
    cv::Vec<T, 3> v{x, y ,z};
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
    cv::Vec<T, 3> v{x, y ,z};
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
inline T Quat<T>::getAngle(bool assumeUnit) const
{
    if (assumeUnit)
    {
        return 2 * std::acos(w);
    }
    if (x * x + y * y + z * z < CV_QUAT_EPS || norm() < CV_QUAT_EPS )
    {
        CV_Error(Error::StsBadArg, "This quaternion does not represent a rotation");
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
cv::Mat Quat<T>::toRotMat4x4(bool assumeUnit) const
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
    cv::Matx<T, 4, 4> R{
          1,                      0,                       0,                   0,
          0,1 - 2 * (c * c + d * d), 2 * (b * c + a * d)    , 2 * (b * d - a * c),
          0,2 * (b * c - a * d)    , 1 - 2 * (b * b + d * d), 2 * (c * d + a * b),
          0,2 * (b * d + a * c)    , 2 * (c * d - a * b)    , 1 - 2 * (b * b + c * c)};
    return cv::Mat(R).t();
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
Vec<T, 3> Quat<T>::toRodrigues(bool assumeUnit) const
{
    if (abs(w) < CV_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "Cannot convert quaternion to rotation vector: rotation vector is indeterminte");
    }
    if (assumeUnit)
    {
        return Vec<T, 3>{x / w, y / w, z / w};
    }
    Quat<T> q = normalize();
    return Vec<T, 3>{q.x / q.w, q.y / q.w, q.z / q.w};

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
Quat<T> Quat<T>::slerp(const Quat<T> &q0, const Quat<T> &q1, const T t, bool assumeUnit, bool directChange)
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
inline Quat<T> Quat<T>::nlerp(const Quat<T> &q0, const Quat<T> &q1, const T t, bool assumeUnit)
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
    // add warning
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
                              const T t, bool assumeUnit,
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
                            const Quat<T> &q2, bool assumeUnit)
{
    Quat<T> v0(q0), v1(q1), v2(q2);
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
        v2 = v2.normalize();
        // add warning in inter
    }
    return v1 * cv::exp(-(cv::log(v1.conjugate() * v0, assumeUnit) + (cv::log(v1.conjugate() * v2, assumeUnit))) / 4);
}

template <typename T>
Quat<T> Quat<T>::spline(const Quat<T> &q0, const Quat<T> &q1, const Quat<T> &q2, const Quat<T> &q3, const T t, bool assumeUnit)
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
    Quat<T> s1 = interPoint(vec[0], vec[1], vec[2], true);
    Quat<T> s2 = interPoint(vec[1], vec[2], vec[3], true);
    return squad(vec[1], s1, s2, vec[2], t, assumeUnit, false);
}

template <typename T>
Quat<T> rvec2Quat(Vec<T, 3> &rvec)
{
    return Quat<T>(rvec);
}

}
