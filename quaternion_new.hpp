#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
namespace cv
{
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
	 */
	template <typename T> 
	class Quat 
	{
	public: 
		// constructor
		Quat() = default;
		explicit Quat(const cv::Vec<T, 4> &coeff);
		Quat(T qw, T qx, T qy, T qz);
		Quat(T angle,const cv::Vec<T, 3> &axis);
		explicit Quat(const cv::Vec<T, 3> &eularAngle);
		explicit Quat(const cv::Mat &rotMat);
		
		
		// get attributes of qutaernion
		// q = ||q||*e^(axis*angle)
		T norm() const;

		T getAngle() const 
		{
			return 2 * acos(coeff[0] / norm())
		}

		cv::Vec<T, 3> getAxis() const 
		{
			T angle = getAngle();
			return cv::Vec<T, 3> {coeff[1], coeff[2], coeff[3]} / (norm() * sin(angle));
		}

		bool isNormal() const;
		void assertNormal() const
		{
			if (!isNormal()) // for example, may add threshold
			{
				throw "Unnormal";
			}
			
		}

		// Unary operation 
		// 1 convertion function
		//  
		cv::Vec<T, 3> toEularAngle() const;
		cv::Mat toRotMat3x3() const
		{
			assertNormal(a);
			// implement here
		}
		cv::Mat toRotMat4x4() const;
		cv::Vec<T, 4> toVec() const;
		
		// 2 basic function
		UnitQuat<T> normalize() const;
		Quat<T> conjugate() const;
		Quat<T> inv() const;
		
		template <typename _T>
		friend Quat<_T> exp(const Quat<_T> &q) const;
		{
			return q.exp();
		}
			
		Quat<T> exp() const
		{
			cv::Vec<T, 3> v{coeff[1], coeff[2], coeff[3]};
			T length = sqrt(v.dot(v));
			T k = sin(length) / length;
			return exp(coeff[0]) * Quat(cos(length), k * v[0], k * v[1], k * v[2]);
		}
		
		template <typename _T>
		friend Quat<_T> log(const Quat<_T> &q) const;
		
		Quat<T> log() const
		{
			cv::Vec<T, 3> v{coeff[1], coeff[2], coeff[3]};
			T length = sqrt(v.dot(v));
			T k = acos(coeff[0] / norm()) / length;
			return Quat(log(length), k * v[0], k * v[1], k * v[2]);
		}

		Quat<T> power(T x) const
		{
			T angle = getAngle();
			cv::Vec<T, 3> axis = getAxis();
			return pow(norm(), x)* Quat(x * angle, axis);
		}
		Quat<T> sinh() const
		Quat<T> cosh() const;
		Quat<T> sin() const;
		Quat<T> cos() const;
		Quat<T> tan() const;
		Quat<T> asin() const;
		Quat<T> acos() const;
		Quat<T> asinh() const;
		Quat<T> acosh() const;
		cv::Vec<T, 4> getCoeff() const;
		

		// binary operation
		cv::Vec<T, 3> crossProduct(const Quat<T> &q) const;
		T dot(Quat<T> &) const;
		
		
		// functional operation
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
		T w, x, y, z;
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


} //namespace cv
