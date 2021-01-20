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
#error This is not a standalone header. Include dualquaternion.hpp instead.
#endif

///////////////////////////////////////////////////////////////////////////////////////
//Implementation
namespace cv {

template <typename T>
DualQuat<T>::DualQuat():w(1), x(0), y(0), z(0), w_(0), x_(0), y_(0), z_(0){};

template <typename T>
DualQuat<T>::DualQuat(const T w, const T x, const T y, const T z, const T w_, const T x_, const T y_, const T z_):
                      w(w), x(x), y(y), z(z), w_(w_), x_(x_), y_(y_), z_(z_){};

template <typename T>
DualQuat<T>::DualQuat(const Vec<T, 8> &q):w(q[0]), x(q[1]), y(q[2]), z(q[3]),
                                          w_(q[4]), x_(q[5]), y_(q[6]), z_(q[7]){};

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
DualQuat<T> DualQuat<T>::createFromAngleAxisTrans(const T angle, const Vec<T, 3> &axis, const Quat<T> &trans)
{
    Quat<T> r = Quat<T>::createFromAngleAxis(angle, axis);
    return createFromQuat(r, trans * r / 2);
}

template <typename T>
DualQuat<T> DualQuat<T>::createFromMat(InputArray _R)
{
    CV_CheckTypeEQ(_R.type(), cv::traits::Type<T>::value, "");

    Mat R = _R.getMat();
    Quat<T> r = Quat<T>::createFromRotMat(R.colRange(0, 3).rowRange(0, 3));
    Quat<T> trans(0, R.at<T>(0, 3), R.at<T>(1, 3), R.at<T>(2, 3));
    return createFromQuat(r, trans * r / 2); 
}

template <typename T>
DualQuat<T> DualQuat<T>::createFromAffine3(const Affine3<T> &R)
{
    return createFromMat(R.matrix);
}

template <typename T>
DualQuat<T> DualQuat<T>::createFromPitch(const T angle, const T d, const Quat<T> &axis, const Quat<T> &moment)
{
    T half_angle = angle / 2, half_d = d / 2;
    Quat<T> qaxis = axis.normalize();
    Quat<T> dual = Quat<T>(-half_d * std::sin(half_angle), 0.0, 0.0, 0.0) + std::sin(half_angle) * moment + 
        half_d * std::cos(half_angle) * qaxis;
    return createFromQuat(Quat<T>::createFromAngleAxis(angle, Vec<T, 3>{qaxis[1], qaxis[2], qaxis[3]}), dual);
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
        return 2.0 * getDualQuat() * getRealQuat().conjugate();
    }
    DualQuat<T> q = normalize();
    return 2.0 * q.getDualQuat() * q.getRealQuat().conjugate();
}

template <typename T>
inline DualQuat<T> DualQuat<T>::normalize() const
{
    Quat<T> p = getRealQuat();
    Quat<T> q = getDualQuat();
    Quat<T> p_n = p / p.norm();
    Quat<T> q_n = q / p.norm();
    return createFromQuat(p_n, q_n - p_n * p_n.dot(q_n));
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
inline DualQuat<T>& DualQuat<T>::operator+=(const DualQuat<T> &q)
{
    *this = *this + q;
    return *this;
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
inline DualQuat<T>& DualQuat<T>::operator*=(const DualQuat<T> &q)
{
    *this = *this * q;
    return *this;
}

template <typename T>
inline DualQuat<T> cv::operator+(const T a, const DualQuat<T> &q)
{
    return DualQuat<T>(a + q.w, q.x, q.y, q.z, q.w_, q.x_, q.y_, q.z_);
}

template <typename T>
inline DualQuat<T> cv::operator+(const DualQuat<T> &q, const T a)
{
    return DualQuat<T>(a + q.w, q.x, q.y, q.z, q.w_, q.x_, q.y_, q.z_);
}

template <typename T>
inline DualQuat<T> cv::operator-(const DualQuat<T> &q, const T a)
{
    return DualQuat<T>(q.w - a, q.x, q.y, q.z, q.w_, q.x_, q.y_, q.z_);
}

template <typename T>
inline DualQuat<T>& DualQuat<T>::operator-=(const DualQuat<T> &q)
{
    *this = *this - q;
    return *this;
}

template <typename T>
inline DualQuat<T> cv::operator-(const T a, const DualQuat<T> &q)
{
    return DualQuat<T>(a - q.w, -q.x, -q.y, -q.z, -q.w_, -q.x_, -q.y_, -q.z_);
}

template <typename T>
inline DualQuat<T> cv::operator*(const T a, const DualQuat<T> &q)
{
    return DualQuat<T>(q.w * a, q.x * a, q.y * a, q.z * a, q.w_ * a, q.x_ * a, q.y_ * a, q.z_ * a);
}

template <typename T>
inline DualQuat<T> cv::operator*(const DualQuat<T> &q, const T a)
{
    return DualQuat<T>(q.w * a, q.x * a, q.y * a, q.z * a, q.w_ * a, q.x_ * a, q.y_ * a, q.z_ * a);
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
inline DualQuat<T>& DualQuat<T>::operator/=(const DualQuat<T> &q)
{
    *this = *this / q;
    return *this;
}

template <typename T>
std::ostream & operator<<(std::ostream &os, const DualQuat<T> &q)
{
    os << "DualQuat " << Vec<T, 8>{q.w, q.x, q.y, q.z, q.w_, q.x_, q.y_, q.z_};
    return os;
}

template <typename _Tp>
Matx<_Tp, 4, 4> DualQuat<_Tp>::Jacobian(const Quat<_Tp> &q) const
{
    _Tp vvT = q.x * q.x + q.y * q.y + q.z * q.z;
    _Tp nv = std::sqrt(vvT);
    _Tp sinc_nv = abs(nv) < CV_DUAL_QUAT_EPS ? 1 - nv * nv / 6 : std::sin(nv) / nv; 
    _Tp csiii_nv = abs(nv) < CV_DUAL_QUAT_EPS ? -(_Tp)1.0 / 3 : (std::cos(nv) - sinc_nv) / nv / nv; 
    Matx<_Tp, 4, 4> J_exp_quat {
        std::cos(nv), -sinc_nv * x,  -sinc_nv * y,  -sinc_nv * z,
        sinc_nv * x, csiii_nv * x * x + sinc_nv, csiii_nv * x * y, csiii_nv * x * z,
        sinc_nv * y, csiii_nv * y * x, csiii_nv * y * y + sinc_nv, csiii_nv * y * z,
        sinc_nv * z, csiii_nv * z * x, csiii_nv * z * y, csiii_nv * z * z + sinc_nv
    };
    return std::exp(q.w) * J_exp_quat;
}

template <typename _Tp>
DualQuat<_Tp> DualQuat<_Tp>::exp() const
{
    /*
    _Tp vvT = x * x + y * y + z * z;
    _Tp nv = std::sqrt(vvT);
    _Tp sinc_nv = abs(nv) < CV_DUAL_QUAT_EPS ? 1 - nv * nv / 6 : std::sin(nv) / nv; 
    _Tp csiii_nv = abs(nv) < CV_DUAL_QUAT_EPS ? -(_Tp)1.0 / 3 : (std::cos(nv) - sinc_nv) / nv / nv; 
    Matx<_Tp, 4, 4> J_exp_quat {
        std::cos(nv), -sinc_nv * x,  -sinc_nv * y,  -sinc_nv * z,
        sinc_nv * x, csiii_nv * x * x + sinc_nv, csiii_nv * x * y, csiii_nv * x * z,
        sinc_nv * y, csiii_nv * y * x, csiii_nv * y * y + sinc_nv, csiii_nv * y * z,
        sinc_nv * z, csiii_nv * z * x, csiii_nv * z * y, csiii_nv * z * z + sinc_nv
    };
    */
    Quat<_Tp> p = getRealQuat();
    return createFromQuat(getRealQuat().exp(), Quat<_Tp>(Jacobian(p) * getDualQuat().toVec()));
}

template <typename T>
DualQuat<T> DualQuat<T>::log(QuatAssumeType assumeUnit) const
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
DualQuat<T> DualQuat<T>::power(const T t, QuatAssumeType assumeUnit) const
{
    return (t * log(assumeUnit)).exp();
}

template <typename T>
DualQuat<T> DualQuat<T>::power(const DualQuat<T> &q, QuatAssumeType assumeUnit) const
{
    return (q * log(assumeUnit)).exp();
}

template <typename T>
Affine3<T> DualQuat<T>::toAffine3() const
{
    return Affine3<T>(toMat());
}

template <typename T>
Matx<T, 4, 4> DualQuat<T>::toMat() const
{
    Matx<T, 4, 4> rot44 = getRotation().toRotMat4x4();
    Quat<T> translation = getTranslation();
    rot44(0, 3) = translation[1];
    rot44(1, 3) = translation[2];
    rot44(2, 3) = translation[3]; 
    return rot44;
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

template <typename T>
DualQuat<T> DualQuat<T>::dqblend(const DualQuat<T> &q1, const DualQuat<T> &q2, const T t, QuatAssumeType assumeUnit)
{
    DualQuat<T> v1(q1), v2(q2);
    if (!assumeUnit)
    {
        v1 = v1.normalize();
        v2 = v2.normalize();
    }
    if (v1.getRotation().dot(v2.getRotation()) < 0)
    {
        return ((1 - t) * v1 - t * v2).normalize();
    }
    return ((1 - t) * v1 + t * v2).normalize();
}

template <typename T>
template <int cn>
DualQuat<T> DualQuat<T>::gdqblend(const Vec<DualQuat<T>, cn> &_dualquat, const Vec<T, cn> &weight)
{
    Vec<DualQuat<T>, cn> dualquat(_dualquat);
    DualQuat<T> dq_blend(0, 0, 0, 0, 0, 0, 0, 0);
    Quat<T> q0 = dualquat[0].getRotation();
    for (size_t i = 0; i < cn; ++i) 
    {
        int k = 1 ? q0.dot(dualquat[i].getRotation()) < 0 : -1;
        dq_blend = dq_blend + dualquat[i] * k * weight[i];
    }
    return dq_blend.normalize();
}


/* template <typename T, int cn> */
/* void dqs(const Matx<T, 3, cn> &in_vert, const Matx<T, 3 cn> &in_normals, */
/*          Matx<T, 3, cn> &out_vert, Matx<T, 3, cn> &out_normals, */
/*          const Vec<DualQuat<T>, cn> &dualquat, const std::vector<std::vector<T>> &weights, */
/*          const std::vector<std::vector<int>> &joint_id) */
template <typename T>
void dqs(const std::vector<Vec<T, 3>> &in_vert, const std::vector<Vec<T, 3>> &in_normals,
         std::vector<Vec<T, 3>> &out_vert, std::vector<Vec<T, 3>> &out_normals,
         const std::vector<DualQuat<T>> &dualquat, const std::vector<std::vector<T>> &weights,
         const std::vector<std::vector<int>> &joint_id)
{
    for (size_t i = 0; i < in_vert.size(); ++i)
    {
        DualQuat<T> dq_blend;
        int k0 = -1;
        const int joint_nu = weights[i].size();
        /* (joint_nu == 0) ? (klq={1,0,0,0}) : (k0 = joint_id[i][0]); */ 
        if (joint_nu == 0)
        {
            dq_blend = {1, 0, 0, 0, 0, 0, 0, 0};
        }
        else
        {
            k0 = joint_id[i][0];
            dq_blend = dualquat[k0] * weights[i][0];
        }
        Quat<T> q0 = dualquat[k0].getRotation();
        for (int j = 1; j < joint_nu; ++j)
        {
            // DualQuat<T> dq = dualquat[joint_id[i][j]];
            const T k = q0.dot(dualquat[i].getRotation()) < 0 ? -1.0 : 1.0;
            dq_blend = dq_blend + dualquat[joint_id[i][j]] * k * weights[i][j];
        }
        dq_blend = dq_blend.normalize();
        Quat<T> p = dq_blend.getRealQuat();
        Quat<T> q = dq_blend.getDualQuat();
        const T a0 = p.w;
        const T ae = q.w;
        Vec<T, 3> d0{p[1], p[2], p[3]};
        Vec<T, 3> de{q[1], q[2], q[3]};
        out_vert.push_back(in_vert[i] + (T)2.0 * (d0.cross(d0.cross(in_vert[i]) + in_vert[i] * a0) + de * a0 - d0 * ae + d0.cross(de)));
        out_normals.push_back(in_normals[i] + (T)2.0 * (d0.cross(d0.cross(in_normals[i]) + a0 * in_normals[i])));
    }
}

template <typename T>
Vec<T, 8> DualQuat<T>::toVec() const
{
    Quat<T> q{1,2,3,4};
    const T a0 = q.w;
    Vec<T, 3> a{q[1], q[2], q[3]};
    std::cout << a0 * a << std::endl;
    return Vec<T, 8>(w, x, y, z, w_, x_, y_, z_);
}

} //namespace 

#endif /*OPENCV_CORE_DUALQUATERNION_INL_HPP*/
