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
		Quaternion<T> t() const;
		/** convert (0, \theta u) to (cos\theta, sin\theta u)
		implementation of exponential function
		*/ 
		UnitQuaternion<T> exp() const;
		T norm() const;

		template <typename _T>	
		static T dot(_T &, _T&);
		//T dot(Quaternion<T>&) const;
		UnitQuaternion<T> normalize() const;
		Quaternion<T> inv() const;
		cv::Vec<T, 4> getCoeff() const;

		T getAngle() const;
		cv::Vec<T, 3> getAxis() const;
		/** @brief get the rotation queternion
		@param angle Rotation angle
		@param axis normalized axis
		 */
		static Quaternion<T> getRotQuat(const T& angle, const cv::Vec<T, 3> &axis);
		template <typename _Tp>
		static bool isNormalized(_Tp obj);
		//这万一输出不同的type呢？   
		// cv::Mat stack(cv::Mat&);
		// cv::Mat stack(cv::Vec<T, 4>);
		
		
		/** @brief transform the position by quaternion
		@param beTransed to be transformed Vec or Mat
		@param T angle rotation angle
		@param axis normalized axis
		*/
		//template <typename _T>
	    // _T static transform(_T &beTransed, const T &angle, const cv::Vec<T, 3> &axis);
	    static void transform(cv::Mat &beTransed, const T &angle, const cv::Vec<T, 3> &axis);

		/** To calculate the transformation between q_0 and q_1 by Spherical Linear Interpolation(Slerp)
		@param q1 a unit Quaternion
		@param q2 a unit Quaternion
		@param t a number in [0, 1]
		*/
		static Quaternion<T> lerp(const Quaternion<T> &q1, const Quaternion &q2, T t);
		
		
		static Quaternion<T> slerp(Quaternion<T> &q1, Quaternion &q2, T t);
		static Quaternion<T> nlerp(const Quaternion<T> &q1, const Quaternion &q2, T t);
		static Quaternion<T> squad(const Quaternion<T> &q1, const Quaternion &q2,
								   const Quaternion<T> &q3, const Quaternion &q4, T t);
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
		UnitQuaternion(T qw, T qx, T qy, T qz);
		cv::Vec<T, 4> getCoeff();
		/** tranform the unit quaternion to matrix form
		 */
		cv::Mat toRotMat33();
		cv::Mat toRotMat44();
		Quaternion<T> log() const;
		static cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&);
		static Quaternion<T> squad(const Quaternion<T> &q1, const Quaternion<T> &q2,
						const Quaternion<T> &q3, const Quaternion<T> &q4, T t);
		UnitQuaternion<T> interPoint(const UnitQuaternion<T> &q0,
						const UnitQuaternion<T> &q1,
						const UnitQuaternion<T> &q2) const;
		UnitQuaternion<T> spline(const UnitQuaternion<T> &q0, const UnitQuaternion<T> &q1,
					    const UnitQuaternion<T> &q20, const UnitQuaternion<T> &q3, T t) const;

				
		UnitQuaternion<T> operator-() const;
		bool operator==(const UnitQuaternion<T>&) const;
		UnitQuaternion<T> operator+(const UnitQuaternion<T>&) const; 
		UnitQuaternion<T>& operator+=(const UnitQuaternion<T>&); 
		UnitQuaternion<T> operator-(const UnitQuaternion<T>&) const; 
		UnitQuaternion<T>& operator-=(const UnitQuaternion<T>&);
		UnitQuaternion<T>& operator*=(const UnitQuaternion<T>&);
		UnitQuaternion<T>& operator*=(const T&);
		UnitQuaternion<T> operator*(const UnitQuaternion<T>&) const;
		Quaternion<T> operator*(Quaternion<T>&) const;
		UnitQuaternion<T> operator/(const T&) const;
		UnitQuaternion<T>& operator/=(const T&);
		T& operator[](std::size_t n);
		const T& operator[](std::size_t n) const;
        template <typename S>
		friend UnitQuaternion<S> operator*(const S, const UnitQuaternion<S>&);
        template <typename S>
		friend UnitQuaternion<S> operator*(const UnitQuaternion<S>&, const S);
		
		template <typename S>
		friend std::ostream& operator<<(std::ostream&, const UnitQuaternion<S>&);
	private:
		cv::Vec<T, 4> coeff{1.0, 0.0, 0.0, 0.0};
	};


//}
