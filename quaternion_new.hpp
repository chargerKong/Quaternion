#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
//namespace cv {
	template <typename T> 
	class UnitQuat;

	template <typename T> 
	class Quat 
	{
	public: 
		Quat() = default;
		explicit Quat(cv::Vec<T, 4> coeff);
		Quat(T qw, T qx, T qy, T qz);
		Quat(T angle, cv::Vec<T, 3> &axis);
		explicit Quat(cv::Vec<T, 3> &eularAngle);
		/**
		 * @brief create a quaternion from a rotation matrix（3 x 3）
		 */
		explicit Quat(cv::Mat&);
		
		T getAngle() const 
		{
			return acos(coeff[0] / norm())
		}

		cv::Vec<T, 3> getAxis() const 
		{
			T angle = getAngle();
			return cv::Vec<T, 3> {coeff[1], coeff[2], coeff[3]} / (norm() * sin(angle));
		}

		Quat<T> conjugate() const;

		/** 
		implementation of exponential function
		*/ 
		static UnitQuat<T> exp(Quat<T> &q)
		{
			return q.exp();
		}

		static UnitQuat<T> log(Quat<T> &q)
		{
			return q.log();
		}

		Quat<T> power(T x) const
		{
			T angle = getAngle();
			cv::Vec<T, 3> axis = getAxis();
			return pow(norm(), x)* Quat(x * angle, axis);

		}

		cv::Vec<T, 3> crossProduct() const;
		Quat<T> exp() const
		{
			cv::Vec<T, 3> v{coeff[1], coeff[2], coeff[3]};
			T length = sqrt(v.dot(v));
			T k = sin(length) / length;
			return exp(coeff[0]) * Quat(cos(length), k * v[0], k * v[1], k * v[2]);
		}

		Quat<T> log() const
		{
			cv::Vec<T, 3> v{coeff[1], coeff[2], coeff[3]};
			T length = sqrt(v.dot(v));
			T k = acos(coeff[0] / norm()) / length;
			return Quat(log(length), k * v[0], k * v[1], k * v[2]);
		}

		Quat<T> sinh() const
		{
			cv::Vec<T, 3> v{coeff[1], coeff[2], coeff[3]};
			T length = sqrt(v.dot(v));
			T k = cosh(coeff[0] / length * sin(length);
			return Quaternion(sinh(coeff[0] * cos(length), k * v[0], k * v[1], k * v[2]); 
		}
		Quat<T> cosh() const;
		{
			cv::Vec<T, 3> v{coeff[1], coeff[2], coeff[3]};
			T length = sqrt(v.dot(v));
			T k = sinh(coeff[0] / length * sin(length);
			return Quaternion(cosh(coeff[0] * cos(length), k * v[0], k * v[1], k * v[2]); 
		}
		Quat<T> sin() const;
		Quat<T> cos() const;
		Quat<T> tan() const;
		Quat<T> asin() const;
		Quat<T> acos() const;
		Quat<T> asinh() const;
		Quat<T> acosh() const;

		cv::Mat toRotMat33() const;
		cv::Mat toRotMat44() const;
		cv::Vec<T, 3> toEularAngle() const;
		T norm() const;
		T dot(Quat<T> &) const;
		UnitQuat<T> normalize() const;
		Quat<T> inv() const;
		cv::Vec<T, 4> getCoeff() const;
		bool isNormal() const;
		void transform44(cv::Mat &beTransed) const;
		void transform33(cv::Mat &beTransed) const;
		static cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&);
		static Quat<T> lerp(const Quat<T> &q1, const Quat &q2,T t);
		static Quat<T> nlerp(const Quat<T> &q1, const Quat &q2, T t);
		/** To calculate the transformation between q_0 and q_1 by Spherical
		 * Linear Interpolation(Slerp)
		@param q1 a unit Quat @param q2 a unit Quat @param t a
		number in [0, 1]
		*/
		static Quat<T> slerp(Quat<T> &q1, Quat &q2, T t);
		static Quat<T> squad(const Quat<T> &q1, const Quat<T> &q2, const Quat<T> &q3, const Quat<T> &q4, T t); 
		Quat<T> interPoint(const Quat<T> &q0, const Quat<T> &q1, const Quat<T> &q2) const;
		Quat<T> spline(consT Quat<T> &q0, const Quat<T> &q1, const Quat<T> &q20, const Quat<T> &q3, T t) const;

		Quat<T> operator-() const;
		bool operator==(const Quat<T>&) const;
		Quat<T> operator+(const Quat<T>&) const;
		Quat<T>& operator+=(const Quat<T>&);
		Quat<T> operator-(const Quat<T>&) const;
		Quat<T>& operator-=(const Quat<T>&);
		Quat<T>& operator*=(const Quat<T>&);
		Quat<T>& operator*=(const T&);
		Quat<T> operator*(const Quat<T>&) const;
		Quat<T> operator/(const T&) const;
		Quat<T>& operator/=(const T&);
		T& operator[](std::size_t n);
		const T& operator[](std::size_t n) const;
		template <typename S>
		friend Quat<S> operator*(const S, const Quat<S>&);
		template <typename S>
		friend Quat<S> operator*(const Quat<S>&, const S);
		template <typename S>
		friend std::ostream& operator<<(std::ostream&, const Quat<S>&);
		
	protected:
		cv::Vec<T, 4> coeff{0.0, 0.0, 0.0, 0.0};
	};


	template <typename T>
	class UnitQuat: public Quat<T>
	{
	public:
		UnitQuat();
		explicit UnitQuat(cv::Mat &R);
		UnitQuat(const Quat<T>&);
		explicit UnitQuat(const UnitQuat<T>&);
		explicit UnitQuat(const cv::Vec<T, 4>&);、
		UnitQuat(const T &angle, cv::Vec<T, 3> &axis);
		UnitQuat(const T qw, const T qx, const T qy, const T qz);
		
		static cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&);
		static Quat<T> lerp(const UnitQuat<T> &q1, const UnitQuat &q2, T t);
		static UnitQuat<T> nlerp(const UnitQuat<T> &q1, const UnitQuat &q2, T t);
		//static void transform(cv::Mat &beTransed, const T &angle, const cv::Vec<T, 3> &axis);
		/** To calculate the transformation between q_0 and q_1 by Spherical Linear Interpolation(Slerp)
		@param q1 a unit Quat
		@param q2 a unit Quat
		@param t a number in [0, 1]
		*/
		static UnitQuat<T> slerp(UnitQuat<T> &q1, UnitQuat &q2, T t);
		static Quat<T> squad(const UnitQuat<T> &q1, const UnitQuat<T> &q2,
						const UnitQuat<T> &q3, const UnitQuat<T> &q4, T t);
		UnitQuat<T> interPoint(const UnitQuat<T> &q0,
						const UnitQuat<T> &q1,
						const UnitQuat<T> &q2) const;
		UnitQuat<T> spline(const UnitQuat<T> &q0, const UnitQuat<T> &q1,
					    const UnitQuat<T> &q20, const UnitQuat<T> &q3, T t) const;

	};


//}
