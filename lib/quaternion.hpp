#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
//namespace cv 
//{
	//template <typename _Tp>
	//class UnitQuat;
	/**
	 * Quaternion are a number system that extends the complex numbers.
	 * A quaternion is generally represented in the form:
	 * 		\begin{array}{rcl}
	 * 		q &= w + x\boldsymbol{i} + y\boldsymbol{j} + z\boldsymbol{k}\\
	 *		  &= [w, x, y, z]\\
	 *		  &= [w, \boldsymbol{v}]
	 *		  \end{array}
	 * A unit quaternion is usually represents rotation, which has the form:
	 * 		q = [cos\theta, sin\theta\cdot u_x, sin\theta\cdot u_y, sin\theta\cdot u_z]
	 *
	 * In fact. A general quaternion could also be represented in the form:
	 * 		q = ||q||[cos\theta, sin\theta\cdot u_x, sin\theta\cdot u_y, sin\theta\cdot u_z]
	 * where ||q|| represents the norm of q
	 * all functions will be degenerate to unit cases when ||q|| = 1
	 */

	template <typename _Tp>
	class Quat
	{
	public:
		Quat() = default;
		explicit Quat(cv::Vec<_Tp, 4> coeff);
		Quat(_Tp qw, _Tp qx, _Tp qy, _Tp qz);
		Quat(const _Tp angle, const cv::Vec<_Tp, 3> &axis);
		explicit Quat(const cv::Mat &R);
		
		/**
		 * @brief get the conjugate of quaternion
		 * q^* = (w, -x, -y, -z)
		 */
		Quat<_Tp> conjugate() const;

		/**
		 * @brief return the value of exponential function
		 * e^q = e^w (cos||v||+ \frac{v}{||v||sin||v||})
		 */
		template <typename T>
		friend Quat<T> exp(Quat<T>&);
		Quat<_Tp> exp() const;

		/**
		 * @brief return the value of logarithm function
		 * ln(q) = ln||q|| + \frac{v}{||v||}arccos\frac{a}{||q||}
		 */
		template <typename T>
		friend Quat<T> log(Quat<T>&);
		Quat<_Tp> log() const;
		
		/**
		 * @brief return the value of power function with constant x
		 * q^x = ||q||(cos(x\theta) + \boldsymbol{v}sin(x\theta)))
		 */
		template <typename T>
		friend Quat<T> power(Quat<T>& q, _Tp x);
		// Quat<_Tp> power(_Tp x) const;

		/**
		 * @brief return the value of power function with quaternion p
		 * q^p = e^{pln(q)}
		 */
		template <typename T>
		friend Quat<T> power(Quat<T> &p, Quat<T> &q);
		//Quat<_Tp> power(Quat<_Tp> p) const;
		
		/**
		 * @brief return the norm of quaternion
		 * ||q|| = \sqrt{w^2 + x^2 + y^2 + z^2}
		 */
		_Tp norm() const;

		/**
		 * @brief return a normalzed q
		 * q_n = \frac{q}{||q||}
		 * where q_n.i satisfy q_n.x^2 + q_n.y^2 + q_n.z^2 + q_n.w^2 = 1
		 */
		Quat<_Tp> normalize() const;

		/**
		 * @brief return an inverse q q^-1
		 * which satisfies q * q^-1 = 1
		 */
		Quat<_Tp> inv() const;

		/**
		 * @brief sinh(p) = sin(w)cos(||v||) + cosh(w)\frac{v}{||v||}sin||v||
		 */
		Quat<_Tp> sinh() const;
		Quat<_Tp> cosh() const;
		Quat<_Tp> tanh() const;
		Quat<_Tp> sin() const;
		Quat<_Tp> cos() const;
		Quat<_Tp> tan() const;
		Quat<_Tp> asin() const;
		Quat<_Tp> acos() const;
		Quat<_Tp> asinh() const;
		Quat<_Tp> acosh() const;
		Quat<_Tp> atanh() const;

		/**
		 * @brirf to dermined whether a quaternion is normalized or not
		 */
		bool isNormal() const;
		
		/**
		 * @brief transform the quaternion q to a 3x3 rotate matrix. The quaternion 
		 * has to be normalized before tranformation
		 */
		cv::Mat toRotMat33() const;

		/**
		 * @brief transform the quaternion q to a 4x4 rotate matrix
		 */
		cv::Mat toRotMat44() const;

		/**
		 * @brief transofrm the quaternion q to a Vec<T, 4>
		 */
		cv::Vec<_Tp, 4> toVec() const;
		
		/**
		* @brief get the angle of quaternion
		* \psi = 2 *arccos(\frac{w}{||q||})
		*/
		_Tp getAngle() const;
		
		/**
		* @brief get the axis of quaternion
		* the unit axis \boldsymbol{n} is defined by
		* \begin{array}
		*      \boldsymbol{v} &= \boldsymbol{n} ||\boldsymbol{v}||\\
		*             &= \boldsymbol{n}||q||sin(\theta)
		*      \end{array}
		*/
		cv::Vec<_Tp, 3> getAxis() const;
		
		/**
		 * @brief return the dot between q and q1
		 * @param q1 a quaternion to be dot
		 * p \cdot q1 = w^2 + x^2 + y^2 + z^2
		 */
		_Tp dot(Quat<_Tp> q1) const;
		
		/**
		 * @ brief return the crossProduct between q and q1
		 */
		cv::Vec<_Tp, 3> crossProduct(Quat<_Tp> q1) const;
		
		
		typedef Quat<float> Quatf;
		typedef Quat<double> Quatd;
		
		static Quat<_Tp> lerp(const Quat<_Tp> &q1, const Quat &q2, _Tp t);
		static Quat<_Tp> nlerp(const Quat<_Tp> &q1, const Quat &q2, _Tp t);
		
		/** _Tpo calculate the interpolation between q_0 and q_1 by Spherical Linear Interpolation(Slerp)
		@param q1 a unit Quat
		@param q2 a unit Quat
		@param t a number in [0, 1]
		*/
		static Quat<_Tp> slerp(const Quat<_Tp> &q1, const Quat &q2, _Tp t);
		static Quat<_Tp> squad(const Quat<_Tp> &q1, const Quat<_Tp> &q2,
						const Quat<_Tp> &q3, const Quat<_Tp> &q4, _Tp t);
		Quat<_Tp> interPoint(const Quat<_Tp> &q0,
						const Quat<_Tp> &q1,
						const Quat<_Tp> &q2) const;
		Quat<_Tp> spline(const Quat<_Tp> &q0, const Quat<_Tp> &q1,
					    const Quat<_Tp> &q2, const Quat<_Tp> &q3, _Tp t) const;
		
		Quat<_Tp> operator-() const;
		bool operator==(const Quat<_Tp>&) const;
		Quat<_Tp> operator+(const Quat<_Tp>&) const; 
		Quat<_Tp>& operator+=(const Quat<_Tp>&); 
		Quat<_Tp> operator-(const Quat<_Tp>&) const; 
		Quat<_Tp>& operator-=(const Quat<_Tp>&);
		Quat<_Tp>& operator*=(const Quat<_Tp>&);
		Quat<_Tp>& operator*=(const _Tp&);
		Quat<_Tp> operator*(const Quat<_Tp>&) const;
		Quat<_Tp> operator/(const _Tp&) const;
		Quat<_Tp>& operator/=(const _Tp&);
		 _Tp& operator[](std::size_t n);
		const _Tp& operator[](std::size_t n) const;
		template <typename S>
		friend Quat<S> operator*(const S, const Quat<S>&);
		template <typename S>
		friend Quat<S> operator*(const Quat<S>&, const S);
		template <typename S>
		friend std::ostream& std::operator<<(std::ostream&, const Quat<S>&);
		
	//protected:
		//cv::Vec<_Tp, 4> coeff{0.0, 0.0, 0.0, 0.0};
		_Tp w, x, y, z;
	};

/*
    template <typename _Tp>
	class UnitQuat: public Quat<_Tp>
	{
	public:
		UnitQuat();
		UnitQuat(const Quat<_Tp>&);
		explicit UnitQuat(const UnitQuat<_Tp>&);
		UnitQuat(const cv::Vec<_Tp, 4>&);
		UnitQuat(const _Tp angle, cv::Vec<_Tp, 3> &axis);
		UnitQuat(const _Tp qw, const _Tp qx, const _Tp qy, const _Tp qz);
		

		static Quat<_Tp> log(UnitQuat<_Tp>&);
		static Quat<_Tp> lerp(const UnitQuat<_Tp> &q1, const UnitQuat &q2, _Tp t);
		static UnitQuat<_Tp> nlerp(const UnitQuat<_Tp> &q1, const UnitQuat &q2, _Tp t);
		/1** _Tpo calculate the transformation between q_0 and q_1 by Spherical Linear Interpolation(Slerp)
		@param q1 a unit Quat
		@param q2 a unit Quat
		@param t a number in [0, 1]
		*1/
		static UnitQuat<_Tp> slerp(UnitQuat<_Tp> &q1, UnitQuat &q2, _Tp t);
		static Quat<_Tp> squad(const UnitQuat<_Tp> &q1, const UnitQuat<_Tp> &q2,
						const UnitQuat<_Tp> &q3, const UnitQuat<_Tp> &q4, _Tp t);
		UnitQuat<_Tp> interPoint(const UnitQuat<_Tp> &q0,
						const UnitQuat<_Tp> &q1,
						const UnitQuat<_Tp> &q2) const;
		UnitQuat<_Tp> spline(const UnitQuat<_Tp> &q0, const UnitQuat<_Tp> &q1,
					    const UnitQuat<_Tp> &q20, const UnitQuat<_Tp> &q3, _Tp t) const;

	};*/
//}//namespace
