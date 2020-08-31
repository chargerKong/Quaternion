#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
//#include <opencv2/core/mat.hpp>
//namespace cv 
//{
	template <typename T>
	class Quaternion
	{
	public:
		Quaternion() = default;
		explicit Quaternion(cv::Vec<T, 4> coeff);
		Quaternion(T qw, T qx, T qy, T qz);	
		explicit Quaternion(cv::Mat &R);

		Quaternion<T> conjugate() const;
		T norm() const;
		T dot(Quaternion<T>&) const;
		Quaternion<T> normalize() const;
		Quaternion<T> inv() const;
		static cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&);
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
		@param _T to be transformed Vec or Mat
		@param T angle rotation angle
		@param axis normalized axis
		*/
		template <typename _T>
		static cv::Mat transform(const _T&, const T&angle, const cv::Mat&);
		
		bool operator==(const Quaternion<T>&) const;
		Quaternion<T> operator+(const Quaternion<T>&) const; 
		Quaternion<T>& operator+=(const Quaternion<T>&); 
		Quaternion<T> operator-(const Quaternion<T>&) const; 
		Quaternion<T>& operator-=(const Quaternion<T>&);
		Quaternion<T>& operator*=(const Quaternion<T>&);
		Quaternion<T>& operator*=(const T&);
		Quaternion<T> operator*(const Quaternion<T>&) const;
		Quaternion<T> operator/(const T&) const;
		Quaternion<T>& operator/=(const T&);
		T& operator[](std::size_t n);
		const T& operator[](std::size_t n) const;
        template <typename T1>
		friend Quaternion<T1> operator*(const T1, const Quaternion<T1>&);
        template <typename T1>
		friend Quaternion<T1> operator*(const Quaternion<T1>&, const T1);
        template <typename T1>
		friend std::ostream& operator<<(std::ostream&, const Quaternion<T1>&);


		cv::Vec<T, 4> coeff{0.0, 0.0, 0.0, 0.0};
	};
//}
