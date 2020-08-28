#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include <opencv2/core/mat.hpp>
//namespace cv 
//{
	class Quaternion
	{
	public:
		Quaternion() = default;
		Quaternion(cv::Vec4d coeff);
		Quaternion(double qw, double qx, double qy, double qz);	
		Quaternion(cv::Mat &R);

		Quaternion conjugate() const;
		double norm() const;
		double dot(Quaternion&) const;
		Quaternion normalize() const;
		Quaternion inv() const; 
	
		bool operator==(const Quaternion&) const;
		Quaternion operator+(const Quaternion&) const; 
		Quaternion& operator+=(const Quaternion&); 
		Quaternion operator-(const Quaternion&) const; 
		Quaternion& operator-=(const Quaternion&);
		Quaternion& operator*=(const Quaternion&);
		Quaternion& operator*=(const double&);
		Quaternion operator*(const Quaternion&) const;
		Quaternion operator/(const double&) const;
		Quaternion& operator/=(const double&);
		double& operator[](std::size_t n);
		const double& operator[](std::size_t n) const;
		friend Quaternion operator*(const double, const Quaternion&);
		friend Quaternion operator*(const Quaternion&, const double);
		friend std::ostream &operator<<(std::ostream&, const Quaternion&);


		cv::Vec4d coeff{0.0, 0.0, 0.0, 0.0};
	};
//}
