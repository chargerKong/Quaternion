#include <limits.h>

using namespace std;
void test_unit_basic()
{
	double angle = CV_PI;
	cv::Vec<double, 3> axis{1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3)};
	cv::Vec<double, 4> coeff{cos(angle / 2), sin(angle / 2)* axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]};
	UnitQuaternion<double> q1;
	UnitQuaternion<double> q2(coeff);
	Quaternion<double> quat(coeff);
	UnitQuaternion<double> q3(quat);
	UnitQuaternion<double> q4(angle, axis);
	UnitQuaternion<double> q5(q2);     //？？？？？？？？
	UnitQuaternion<double> q6(cos(angle / 2), sin(angle / 2)* axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]);
	UnitQuaternion<double> q7(7,8,9,10);
	cout << q1.getCoeff() << endl;
	cout << q2.getCoeff() << endl;
	cout << q1 + q4 << endl;
	cout << q4 - q1<< endl;
	q7 += q1;
	cout << q7 << endl;
	q7 -= q1;
	cout << q7 << endl;
	
	
	double scalar = 4;
	cout << scalar * q7 << endl;
	cout << q7 * scalar << endl;
	
	Quaternion<double> q77(7,8,9,10);
	Quaternion<double> q22(cos(angle / 2), sin(angle / 2)* axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]);
	cout << q7 * q2 << endl;
	cout << q77 * q22 <<endl;
	q7 /= scalar;
	cout << q7 << endl;
	
	cout << -q7 << endl;
		
	cout << numeric_limits<double>::min() << endl;
	
	Quaternion<double> q8(0, angle * axis[0], angle * axis[1], angle * axis[2]);
	cout << q8 << endl;
	cout << q8.exp() << endl;
	cout << q8.exp().log() << endl;
	
}
