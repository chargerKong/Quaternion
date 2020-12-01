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
#ifndef OPENCV_CORE_DUALQUATERNION_HPP
#define OPENCV_CORE_DUALQUATERNION_HPP 

#include <opencv2/core/quaternion.hpp>

namespace cv{
//! @addtogroup core
//! @{

template <typename _Tp> class Quat;
template <typename _Tp> std::ostream& operator<<(std::ostream&, const Quat<_Tp>&);

// how to rotate a line and a point
template <typename _Tp>
class DualQuat{
    static_assert(std::is_floating_point<_Tp>::value, "Dual quaternion only make sense with type of float or double");
    using value_type = _Tp;

public:
    static constexpr _Tp CV_DUAL_QUAT_EPS = (_Tp)1.e-6;

    DualQuat();

    /**
     * @brief create from eight same type number
     */
    DualQuat(_Tp w, _Tp x, _Tp y, _Tp z, _Tp w_, _Tp x_, _Tp y_, _Tp z_);
    

    _Tp w, x, y, z, w_, x_, y_, z_;

    /**
     * @brief create Dual Quaternion from two same type quaternions p and q.
     * A Dual Quaternion \f$\sigma\f$ has the form:
     * \f[\sigma = p + \epsilon q\f]
     * where p and q are defined as:
     * \f[\begin{equation}
     *    \begin{split}
     *    p &= w + x\boldsymbol{i} + y\boldsymbol{j} + z\boldsymbol{k}\\
     *    q &= w\_ + x\_\boldsymbol{i} + y\_\boldsymbol{j} + z\_\boldsymbol{k}.
     *    \end{split}
     *   \end{equation}
     * \f]
     * The p and q are the real part and dual part respectively.
     * @parm realPart a quaternion, real part of dual quaternion.
     * @parm duaPlart a quaternion, dual part of dual quaternion.
     * @sa Quat
    */
    static DualQuat<_Tp> createFromQuat(const Quat<_Tp> &realPart, const Quat<_Tp> &dualPart);

    /**
     * @brief create a dual quaternion from a rotation angle \f$\thet\f$, a rotation axis
     * \f$\boldsymbol{u}\f$ and a translation \f$\boldsymbol{t}\f$.
     * it generates a dual quaternion \f$\sigma\f$ in the form of
     * \f[\begin{equation}
     *    \begin{split}
     *    \sigma &= r + \frac{\epsilon}{2}\boldsymbol{t}r \\
     *           &= [\cos(\frac{\theta}{2}), \boldsymbol{u}\sin(\frac{\theta}{2})]
     *           + \frac{\epsilon}{2}[0, \boldsymbol{t}][[\cos(\frac{\theta}{2}),
     *           \boldsymbol{u}\sin(\frac{\theta}{2})]]\\
     *           &= \cos(\frac{\theta}{2}) + \boldsymbol{u}\sin(\frac{\theta}{2})
     *           + \frac{\epsilon}{2}(-(\boldsymbol{t} \cdot \boldsymbol{u})\sin(\frac{\theta}{2})
     *           + \boldsymbol{t}\cos(\frac{\theta}{2}) + \boldsymbol{u} \times \boldsymbol{t} \sin(\frac{\theta}{2})).
     *    \end{split}
     *    \end{equation}\f]
     * @param angle rotation angle.
     * @param axis a normalized rotation axis.
     * @param translation a pure quaternion.
     * @note axis will be normalized in this function. And translation will be applied
     * after the rotation.
     * @sa Quat
     */
    static DualQuat<_Tp> createFromAngleAxisTrans(_Tp angle, const Vec<_Tp, 3> &axis, const Quat<_Tp> &translation);

    /**
     * @brief return a quaternion which represent the real part of dual quaternion.
     * The definition of real part is in createFromQuat().
     * @sa createFromQuat, getDualQuat
     */
    Quat<_Tp> getRealQuat() const;
    
    /**
     * @brief return a quaternion which represent the dual part of dual quaternion.
     * The definition of dual part is in createFromQuat().
     * @sa createFromQuat, getRealQuat
     */
    Quat<_Tp> getDualQuat() const;

    /**
     * @brief return the conjugate of a dual quaternion.
     * \f[
     * \begin{equation}
     * \begin{split}
     * \sigma^* &= (p + \epsilon q)^*
     *          &= (p^* + \epsilon q^*)
     * \end{split}
     * \end{equation}
     * \f]
     */
    DualQuat<_Tp> conjugate() const;

    /**
     * @brief return the rotation in quaternion form.
     */
    Quat<_Tp> getRotation(QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT) const;

    /**
     * @brief return the translation in pure quaternion form.
     * The rotation \f$r\f$ in this dual quaternion \f$\sigma\f$ is applied before translation \f$\boldsymbol{v}t\f$,
     * which has the form
     * \f[\begin{eqaution}
     * \begin{split}
     * \sigma &= p + \epsilon q \\
     *        &= r + \frac{\epsilon}{2}\boldsymbol{t}r
     * \end{split}
     * \end{equation}.\f]
     * Thus, the translation can be obtained as:
     * \f[\boldsymbol{v} = 2 * q * p^*.\f]
     */
    Quat<_Tp> getTranslation(QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT) const;

    /**
     * @brief return the norm \f$||A||\f$ of dual quaternion \f$A = p + \epsilon q\f$. 
     * \f[ 
     *  \begin{equation}
     *  \begin{split}
     *  ||A|| &= \sqrt{A * A^*} \\
     *        &= ||p|| + \epsilon \frac{p \cdot q}{||p||}\f].
     *  \end{split}
     *  \end{equation}
     * Generally speaking, the norm of a not unit dual 
     * quaternion is a dual number. For convenience, we return it in the form of a dual quaternion
     * , i.e. 
     * \f[ ||A|| = [||p||, 0, 0, 0, \frac{p \cdot q}{||p||}, 0, 0, 0, 0].\f]
     *
     * @note The data type of dual number is dual quaternion.
     * @param assumeUnit if QUAT_ASSUME_UNIT, this dual quaternion assume to be a unit quaternion and this f, this dual quaternion 
     */
    DualQuat<_Tp> norm(QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT) const;

    /**
     * @brief return a normalized dual quaternion.
     * A dual quaternion can be expressed as
     * \f[
     * \begin{equation}
     * \begin{split}
     * \sigma &= p + \epsilon q\\
     * &=||\sigma||\left(r+\frac{1}{2}tr\right)
     * \end{split}
     * \end{equation}
     * \f]
     * where \f$r, t\f$ represents the rotation (ordinary quaternion) and translation (pure ordinary quaternion) respectively, 
     * and \f[||\sigma||\f$\f$ is the norm of dual quaternion(a dual number). 
     * A dual quaternion is unit if and only if 
     * \f[
     * ||p||=1, p \cdot q=0
     * \f]
     * where \f$\cdot\f$ means dot product.
     * The process of normalization is 
     * \f[
     * \sigma_{u}=\frac{\sigma}{||\sigma||}
     * \f]
     * Next, we simply proof \f[\sigma_u\f$\f$ is a unit dual quaternion:
     * \f[
     * \renewcommand{\Im}{\operatorname{Im}}
     * \begin{equation}
     * \begin{split}
     * \sigma_{u}=\frac{\sigma}{||\sigma||}&=\frac{p + \epsilon q}{||p||+\epsilon\frac{p\cdot q}{||p||}}\\
     * &=\frac{p}{||p||}+\epsilon\left(\frac{q}{||p||}-p\frac{p\cdot q}{||p||^3}\right)\\
     * &=\frac{p}{||p||}+\epsilon\frac{1}{||p||^2}\left(qp^{*}-p\cdot q\right)\frac{p}{||p||}\\
     * &=\frac{p}{||p||}+\epsilon\frac{1}{||p||^2}\Im(qp^*)\frac{p}{||p||}.\\
     * \end{split}
     * \end{equation}
     * \f]
     * As expected, the real part is a rotation and dual part is a pure quaternion.
     */
    DualQuat<_Tp> normalize() const;

    /**
     * @brief if \f$\signma = p + \epsilon q\f$ is a dual quaternion, p is not zero,
     * then the inverse dual quaternion is
     * \f[\sigma^{-1} = \frac{\sigma^*}{||\sigma||^2}, \f]
     * or equivalentlly,
     * \f[\sigma^{-1} = p^{-1} - \epsilon p^{-1}qp^{-1}.\f]
     */
    DualQuat<_Tp> inv(QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT) const;

    /**
     * @brief 
     */
    _Tp dot(DualQuat<_Tp> p) const;
    
    /**
     * @brief A dual quaternion is a vector in form of
     * \f[
     * \begin{equation}
     * \begin{split}
     * \sigma &=\boldsymbol{p} + \epsilon \boldsymbol{q}\\
     * &= \cos\hat{\frac{\theta}{2}}+\overline{\hat{l}}\sin\frac{\hat{\theta}}{2}
     * \end{split}
     * \end{equation}
     * \f]
     * where \f$\hat{\theta}\f$ is dual angle and \f$\overline{\hat{l}}\f$ is dual axis:
     * \f[
     * \hat{\theta}=\theta + \epsilon d,\\
     * \overline{\hat{l}}= \hat{l} +\epsilon m.
     * \f]
     * In this representation, \f$\theta\f$ is rotation angle and \f$(\hat{l},m)\f$ is the screw axis, d is the translation distance along the axis.
     * 
     * A convenience form of power function of dual quaternion can be expressed as:
     * \f[
     * \sigma^t = \cos\hat{\frac{t\theta}{2}}+\overline{\hat{l}}\sin\frac{\hat{t\theta}}{2}
     * \f]
     * Obviously,  this operation keeps the same screw axis and scales with both rotation angle and translation distance.
     */
    DualQuat<_Tp> power(const _Tp t, const QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT) const; 
    
    DualQuat<_Tp> power(const DualQuat<_Tp>&) const;

    /**
     * @brief A dual quaternion is a vector in form of 
     * \f[
     * \begin{equation}
     * \begin{split}
     * \sigma &=\boldsymbol{p} + \epsilon \boldsymbol{q}\\
     * &= [\hat{q}_0,\hat{q}_1,\hat{q}_2,\hat{q}_3]\\
     * &= [\hat{q}_0,\boldsymbol{v}]
     * \end{split}
     * \end{equation}
     * \f]
     * where \f$\hat{q}_i = p_i+\epsilon q_i\f$. \f$p_i, q_i\f$ is the element of \f$\boldsymbol{p},\boldsymbol{q}\f$ respectively.
     * 
     * Thus, the exponential function of a dual quaternion can be calculated as the method of a quaternion:
     * \f[
     * \exp(\sigma)=e^{\hat{q}_0}\left(\cos||\boldsymbol{v}||+\frac{\boldsymbol{v}}{||\boldsymbol{v}||}\sin||\boldsymbol{v}||\right)
     * \f]
     * To calculate \f$e^{\hat{q}_0}\f$, we expand \f$e^{\hat{q}_0}\f$ by Taylor series:
     * \f[
     * \begin{equation}
     * \begin{split}
     * e^{\hat{q}_0} &= e^{p_0+\epsilon q_0}\\
     * &=e^{p_0}+\epsilon q_0e^{p_0}
     * \end{split}
     * \end{equation}
     * \f]
     * and the same operations for \f$\cos\f$ and \f$\sin.\f$
     */
    DualQuat<_Tp> exp() const;

    /**
     * @brief
     * A dual quaternion is a vector in form of
     * \f[
     * \begin{equation}
     * \begin{split}
     * \sigma &=\boldsymbol{p} + \epsilon \boldsymbol{q}\\
     * &= [\hat{q}_0,\hat{q}_1,\hat{q}_2,\hat{q}_3]\\
     * &= [\hat{q}_0,\boldsymbol{v}]
     * \end{split}
     * \end{equation}
     * \f]
     * where \f$\hat{q}_i = p_i+\epsilon q_i\f$. \f$p_i, q_i\f$ is the element 
     * of \f$\boldsymbol{p},\boldsymbol{q}\f$ respectively.
     * 
     * Thus, the logarithm function of a dual quaternion can be calculated as the method of a quaternion:
     * \f[
     * \ln(\sigma)=\ln||\sigma||+\frac{\boldsymbol{v}}{||\boldsymbol{v}||}\arccos\frac{\hat{q}_0}{\boldsymbol{||v||}}
     * \f]
     * To calculate each function, we expand them by Taylor series, see exp for example.
     *
     * @param assumeUnit if QUAT_ASSUME_UNIT, this quaternion  assume to be a unit quaternion 
     * and this function will save some computations.
     */ 
    DualQuat<_Tp> log(const QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT) const;

    static DualQuat<_Tp> sclerp(const DualQuat<_Tp> &q1, const DualQuat<_Tp> &q2, const _Tp t, bool directChange=true, QuatAssumeType assumeUnit=QUAT_ASSUME_NOT_UNIT);
    
    bool operator==(const DualQuat<_Tp>&) const;
    DualQuat<_Tp> operator-(const DualQuat<_Tp>&) const;
    DualQuat<_Tp> operator-() const;
    DualQuat<_Tp> operator+(const DualQuat<_Tp>&) const;
    DualQuat<_Tp> operator*(const DualQuat<_Tp>&) const;
    
    template <typename S, typename T>
    friend DualQuat<S> cv::operator*(const DualQuat<S>&, const T a);
    
    DualQuat<_Tp> operator/(const _Tp a) const;
    DualQuat<_Tp> operator/(const DualQuat<_Tp>&) const;

    template <typename S>
    friend std::ostream& cv::operator<<(std::ostream&, const DualQuat<S>&);

private:
    static DualQuat<_Tp> createFromPitch(const _Tp angle, const _Tp d, const Quat<_Tp> &axis, const Quat<_Tp> &moment);
};


using DualQuatd = DualQuat<double>;
using DualQuatf = DualQuat<float>;

//! @} core
}//namespace

#include "dualquaternion.inl.hpp"

#endif /* OPENCV_CORE_QUATERNION_HPP */
