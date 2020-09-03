#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
//#include <opencv2/core/mat.hpp>
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

		Quaternion<T> conjugate() const;
		/** convert (0, \theta u) to (cos\theta, sin\theta u)
		implementation of exponential function
		*/ 
		static UnitQuaternion<T> exp(Quaternion<T>&);
		T norm() const;

		
		T dot(Quaternion<T> &) const;
		//T dot(Quaternion<T>&) const;
		UnitQuaternion<T> normalize() const;
		Quaternion<T> inv() const;
		cv::Vec<T, 4> getCoeff() const;

		
		/** @brief get the rotation queternion
		@param angle Rotation angle
		@param axis normalized axis
		 */
		static UnitQuaternion<T> getRotQuat(const T& angle, const cv::Vec<T, 3> &axis);
		
		/*@ brief _T could be Vec or Quaternion
		 */ 
		template <typename _T>
		static bool isNormalized(_T&);
		
		/** @brief transform the position by quaternion
		@param beTransed to be transformed Vec or Mat
		@param T angle rotation angle
		@param axis normalized axis
		*/
		//template <typename _T>
	    // _T static transform(_T &beTransed, const T &angle, const cv::Vec<T, 3> &axis);
	    static void transform(cv::Mat &beTransed, const T &angle, const cv::Vec<T, 3> &axis);


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
		cv::Vec<T, 4> coeff{2.0, 0.0, 0.0, 0.0};
	};


    template <typename T>
	class UnitQuaternion: public Quaternion<T>
	{
	public:
		UnitQuaternion();
		explicit UnitQuaternion(cv::Mat &R);
		UnitQuaternion(const Quaternion<T>&);
		UnitQuaternion(const UnitQuaternion<T>&);
		UnitQuaternion(const cv::Vec<T, 4>&);
		UnitQuaternion(const T &angle, cv::Vec<T, 3> &axis);
		UnitQuaternion(const T qw, const T qx, const T qy, const T qz);
		/** tranform the unit quaternion to matrix form
		 */
		T getAngle() const;
		cv::Vec<T, 3> getAxis() const;
		
		cv::Mat toRotMat33();
		cv::Mat toRotMat44();

		static Quaternion<T> log(UnitQuaternion<T>&);
		static cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&);
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


//}
