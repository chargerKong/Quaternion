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
		cv::Mat getRotMat(const T&, const cv::Vec<T, 3>&) const;

		//这万一输出不同的type呢？
		void transform(const T&, const cv::Vec<T, 3>&, T&);

		
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
		cv::Mat R;
	};
//}
