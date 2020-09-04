#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
//namespace cv 
//{
	template <typename T>
	class UnitQuaternion;

	template <typename T>
	class Quaternion
	{
	public:
		Quaternion() = default;
		explicit Quaternion(cv::Vec<T, 4> coeff);
		Quaternion(T qw, T qx, T qy, T qz);
		Quaternion(const T angle, const cv::Vec<T, 3> &axis);

		Quaternion<T> conjugate() const;
		static UnitQuaternion<T> exp(Quaternion<T>&);
		T norm() const;
		T dot(Quaternion<T> &) const;
		UnitQuaternion<T> normalize() const;
		Quaternion<T> inv() const;
		cv::Vec<T, 4> getCoeff() const;
		/** @brief get the rotation queternion
		@param angle Rotation angle
		@param axis normalized axis
		 */
		bool isNormal() const;
		
		Quaternion<T> operator-() const;
		bool operator==(const Quaternion<T>&) const;
		Quaternion<T> operator+(const Quaternion<T>&) const; 
		Quaternion<T>& operator+=(const Quaternion<T>&); 
		Quaternion<T> operator-(const Quaternion<T>&) const; 
		Quaternion<T>& operator-=(const Quaternion<T>&);
		Quaternion<T>& operator*=(const Quaternion<T>&);
		Quaternion<T>& operator*=(const T&);
		Quaternion<T> operator*(const Quaternion<T>&) const;
		Quaternion<T> operator*(UnitQuaternion<T>&) const;
		Quaternion<T> operator/(const T&) const;
		Quaternion<T>& operator/=(const T&);
		T& operator[](std::size_t n);
		const T& operator[](std::size_t n) const;
		template <typename S>
		friend Quaternion<S> operator*(const S, const Quaternion<S>&);
		template <typename S>
		friend Quaternion<S> operator*(const Quaternion<S>&, const S);
		template <typename S>
		friend std::ostream& operator<<(std::ostream&, const Quaternion<S>&);
		
	protected:
		cv::Vec<T, 4> coeff{0.0, 0.0, 0.0, 0.0};
	};


    template <typename T>
	class UnitQuaternion: public Quaternion<T>
	{
	public:
		UnitQuaternion();
		explicit UnitQuaternion(cv::Mat &R);
		UnitQuaternion(const Quaternion<T>&);
		explicit UnitQuaternion(const UnitQuaternion<T>&);
		UnitQuaternion(const cv::Vec<T, 4>&);
		UnitQuaternion(const T angle, cv::Vec<T, 3> &axis);
		UnitQuaternion(const T qw, const T qx, const T qy, const T qz);
		T getAngle() const;
		cv::Vec<T, 3> getAxis() const;
		
		cv::Mat toRotMat33();
		cv::Mat toRotMat44();

		static Quaternion<T> log(UnitQuaternion<T>&);
		/** tranform the unit quaternion to matrix form
		 */
		// static cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&);
		static Quaternion<T> lerp(const UnitQuaternion<T> &q1, const UnitQuaternion &q2, T t);
		static UnitQuaternion<T> nlerp(const UnitQuaternion<T> &q1, const UnitQuaternion &q2, T t);
		/** To calculate the transformation between q_0 and q_1 by Spherical Linear Interpolation(Slerp)
		@param q1 a unit Quaternion
		@param q2 a unit Quaternion
		@param t a number in [0, 1]
		*/
		static UnitQuaternion<T> slerp(UnitQuaternion<T> &q1, UnitQuaternion &q2, T t);
		static Quaternion<T> squad(const UnitQuaternion<T> &q1, const UnitQuaternion<T> &q2,
						const UnitQuaternion<T> &q3, const UnitQuaternion<T> &q4, T t);
		UnitQuaternion<T> interPoint(const UnitQuaternion<T> &q0,
						const UnitQuaternion<T> &q1,
						const UnitQuaternion<T> &q2) const;
		UnitQuaternion<T> spline(const UnitQuaternion<T> &q0, const UnitQuaternion<T> &q1,
					    const UnitQuaternion<T> &q20, const UnitQuaternion<T> &q3, T t) const;

	};
