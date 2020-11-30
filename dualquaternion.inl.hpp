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

#ifndef OPENCV_CORE_DUALQUATERNION_INL_HPP
#define OPENCV_CORE_DUALQUATERNION_INL_HPP

#ifndef OPENCV_CORE_DUALQUATERNION_HPP
#erorr This is not a standalone header. Include dualquaternion.hpp instead.
#endif

///////////////////////////////////////////////////////////////////////////////////////
//Implementation
namespace cv {

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
DualQuat<T> DualQuat<T>::createFromPitch(const T angle, const T d, const Quat<T> &axis, const Quat<T> &moment)
{
    T half_angle = angle / 2, half_d = d / 2;
    Quat<T> dual = Quat<T>(-half_d * std::sin(half_angle), 0.0, 0.0, 0.0) + std::sin(half_angle) * moment + half_d * std::cos(half_angle) * axis;
    return createFromQuat(Quat<T>::createFromAngleAxis(angle, Vec<T, 3>{axis[1], axis[2], axis[3]}), dual);
}

template <typename T>
inline bool DualQuat<T>::operator==(const DualQuat<T> &q) const
{
    return (abs(w - q.w) < CV_DUAL_QUAT_EPS && abs(x - q.x) < CV_DUAL_QUAT_EPS &&
            abs(y - q.y) < CV_DUAL_QUAT_EPS && abs(z - q.z) < CV_DUAL_QUAT_EPS &&
            abs(w_ - q.w_) < CV_DUAL_QUAT_EPS && abs(x_ - q.x_) < CV_DUAL_QUAT_EPS &&
            abs(y_ - q.y_) < CV_DUAL_QUAT_EPS && abs(z_ - q.z_) < CV_DUAL_QUAT_EPS);
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

template<typename T>
inline DualQuat<T> DualQuat<T>::conjugate() const
{
    return DualQuat<T>(w, -x, -y, -z, w_, -x_, -y_, -z_);
}

template <typename T>
DualQuat<T> DualQuat<T>::norm(QuatAssumeType assumeUnit) const
{
    Quat<T> real = getRealQuat();
    T realNorm = real.norm();
    if (assumeUnit)
    {
        return DualQuat<T>(realNorm, 0, 0, 0, 0, 0, 0, 0); 
    }
    Quat<T> dual = getDualQuat();
    if (realNorm < CV_DUAL_QUAT_EPS){
        return DualQuat<T>(0, 0, 0, 0, 0, 0, 0, 0);
    }
    return DualQuat<T>(realNorm, 0, 0, 0, real.dot(dual) / realNorm, 0, 0, 0);
}

template <typename T>
inline Quat<T> DualQuat<T>::getRotation(QuatAssumeType assumeUnit) const
{
    if (assumeUnit)
    {
        return getRealQuat();
    }
    return getRealQuat().normalize();
}

template <typename T>
Quat<T> DualQuat<T>::getTranslation(QuatAssumeType assumeUnit) const
{
    if (assumeUnit)
    {
        return 2 * getDualQuat() * getRealQuat().conjugate();
    }
    DualQuat<T> q = normalize();
    return 2 * q.getDualQuat() * q.getRealQuat().conjugate();
}

template <typename T>
inline DualQuat<T> DualQuat<T>::normalize() const
{
    DualQuat<T> qNorm = norm();
    if (qNorm.w < CV_DUAL_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "Cannot normalize this dual quaternion: the norm is too small.");
    }
    return *this / qNorm;
}

template <typename T>
inline T DualQuat<T>::dot(DualQuat<T> q) const
{
    return q.w * w + q.x * x + q.y * y + q.z * z + q.w_ * w_ + q.x_ * x_ + q.y_ * y_ + q.z_ * z_;
}

template <typename T>
inline DualQuat<T> DualQuat<T>::inv(QuatAssumeType assumeUnit) const
{
    if (assumeUnit){
        return conjugate();
    }
    DualQuat<T> qNorm = norm();
    if (qNorm.w < CV_DUAL_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "This dual quaternion do not have inverse dual quaternion");
    }
    DualQuat<T> qNorm2 = qNorm * qNorm;
    DualQuat<T> conj = conjugate();
    Quat<T> p = conj.getRealQuat();
    return createFromQuat(p / qNorm2.w, -p * qNorm2.w_ / (qNorm2.w * qNorm2.w ) + conj.getDualQuat() / qNorm2.w);
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

template <typename T, typename S>
DualQuat<T> cv::operator*(const DualQuat<T> &q, const S a)
{
    static_assert(std::is_same<S, int>::value ||
                 std::is_same<S, double>::value ||
                 std::is_same<S, float>::value , "ni zhey buxing ");
    return DualQuat<T>{q.w * a, q.x * a, q.y * a, q.z * a, q.w_ * a, q.x_ * a, q.y_ * a, q.z_ * a};
}

template <typename T>
inline DualQuat<T> DualQuat<T>::operator/(const T a) const
{
    return DualQuat<T>(w / a, x / a, y / a, z / a, w_ / a, x_ / a, y_ / a, z_ / a);
}

template <typename T>
inline DualQuat<T> DualQuat<T>::operator/(const DualQuat<T> &q) const
{
    return *this * q.inv();
}

template <typename T>
std::ostream & operator<<(std::ostream &os, const DualQuat<T> &q)
{
    os << "DualQuat " << Vec<T, 8>{q.w, q.x, q.y, q.z, q.w_, q.x_, q.y_, q.z_};
    return os;
}

template <typename T>
DualQuat<T> DualQuat<T>::exp() const
{
    DualQuat<T> v(0, x, y, z, 0, x_, y_, z_);
    DualQuat<T> normV = v.norm();
    DualQuat<T> sin_normV(std::sin(normV.w), 0, 0, 0, normV.w_ * std::cos(normV.w), 0, 0, 0);
    DualQuat<T> cos_normV(std::cos(normV.w), 0, 0, 0, -normV.w_ * std::sin(normV.w), 0, 0, 0);
    DualQuat<T> exp_w(std::exp(w), 0, 0, 0, w_ * std::exp(w), 0, 0, 0);
    if (normV.w < CV_DUAL_QUAT_EPS)
    {
        return exp_w * (cos_normV + v);
    }
    DualQuat<T> k = sin_normV / normV;
    return exp_w * (cos_normV + v * k);
}

template <typename T>
DualQuat<T> DualQuat<T>::log(const QuatAssumeType assumeUnit) const
{
    DualQuat<T> v(0, x, y, z, 0, x_, y_, z_);
    DualQuat<T> normV = v.norm();
    if (assumeUnit)
    {
        if (normV.w < CV_DUAL_QUAT_EPS)
        {
            return v;
        }
        DualQuat<T> k = DualQuat<T>(std::acos(w), 0, 0, 0, -w_ * 1.0 / std::sqrt(1 - w * w), 0, 0, 0) / normV;
        return v * k;
    }
    DualQuat<T> qNorm = norm();
    if (qNorm.w < CV_DUAL_QUAT_EPS)
    {
        CV_Error(Error::StsBadArg, "Cannot apply this quaternion to log function: undefined");
    }
    DualQuat<T> log_qNorm(std::log(qNorm.w), 0, 0, 0, qNorm.w_ / qNorm.w, 0, 0, 0);
    if (normV.w < CV_DUAL_QUAT_EPS)
    {
        return log_qNorm + v;
    }
    DualQuat<T> coeff = DualQuat<T>(w, 0, 0, 0, w_, 0, 0, 0) / qNorm;
    DualQuat<T> k = DualQuat<T>{std::acos(coeff.w), 0, 0, 0, -coeff.w_ / std::sqrt(1 - coeff.w * coeff.w), 0, 0, 0} / normV;
    return log_qNorm + v * k;
}

template <typename T>
template <typename _T>
DualQuat<T> DualQuat<T>::power(const _T t, const QuatAssumeType assumeUnit) const
{
    Quat<T> p = getRealQuat();
    T angle = p.getAngle(assumeUnit);
    DualQuat<T> qNorm = norm();
    if (abs(angle) < CV_DUAL_QUAT_EPS)
    {
        return DualQuat<T>(std::pow(qNorm.w, t), 0, 0, 0, qNorm.w_ * std::pow(t * qNorm.w, t - 1), 0, 0, 0);
    }
    Vec<T, 3> axis = p.getAxis(assumeUnit);
    Quat<T> qaxis{0, axis[0], axis[1], axis[2]};
    Quat<T> distance = getDualQuat() * p.conjugate() * 2;
    Quat<T> m = (distance.crossProduct(qaxis) + qaxis.crossProduct(distance.crossProduct(qaxis))
                 * std::cos(angle / 2)/std::sin(angle / 2)) / 2;
    if (assumeUnit)
    {
        return createFromPitch(angle * t, distance.dot(qaxis) * t, qaxis, m);
    }
    return DualQuat<T>(std::pow(qNorm.w, t), 0, 0, 0, qNorm.w_ * std::pow(t * qNorm.w, t - 1), 0, 0, 0) * createFromPitch(angle * t, distance.dot(qaxis) * t, qaxis, m);
}

template <typename T>
DualQuat<T> DualQuat<T>::sclerp(const DualQuat<T> &q0, const DualQuat<T> &q1, const T t, bool directChange, QuatAssumeType assumeUnit)
{
    DualQuat<T> v0(q0), v1(q1);
    if (!assumeUnit)
    {
        v0 = v0.normalize();
        v1 = v1.normalize();
    }
    Quat<T> v0Real = v0.getRealQuat();
    Quat<T> v1Real = v1.getRealQuat();
    if (directChange && v1Real.dot(v0Real) < 0)
    {
        v0 = -v0;
    }
    DualQuat<T> v0inv1 = v0.inv() * v1;
    return v0 * v0inv1.power(t, QUAT_ASSUME_UNIT);
}

} //namespace 

#endif /*OPENCV_CORE_DUALQUATERNION_INL_HPP*/
