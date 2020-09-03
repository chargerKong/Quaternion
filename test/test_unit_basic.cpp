#include <limits.h>
using namespace std;
void test_unit_basic()
{
	double angle =1 * CV_PI  ;
	cv::Vec<double, 3> axis{1 / sqrt(3), 1 / sqrt(3), 1 / sqrt(3)};
	cv::Vec<double, 4> coeff{cos(angle / 2), sin(angle / 2)* axis[0], sin(angle / 2) * axis[1], sin(angle / 2) * axis[2]};
	UnitQuaternion<double> q1;
	UnitQuaternion<double> q2(coeff);
	Quaternion<double> quat(1,2,3,4);
	Quaternion<double> quat_q2(coeff);
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
	cout << "Quaternion * UnitQuaternion\n" << quat * q2 << endl;
	assert((quat * quat_q2).getCoeff() == (quat * q2).getCoeff());
	cout << "UnitQuaternion * Quaternion\n" << q2 * quat_q2 << endl;
	assert((q2 * q2).getCoeff() == (q2 * quat_q2).getCoeff());
	
	
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
		
	
	cout << "\ntest for exp and log" << endl;
	Quaternion<double> q8(0, angle * axis[0], angle * axis[1], angle * axis[2]);
	cout << "q8 is: \t\t" << q8 << endl;
	cout << "q8.exp() is : \t" << q8.exp() << endl;
	cout << "q8.exp().log() is :" << q8.exp().log() << endl;
	
	//UnitQuaternion<double> q2;
	cv::Mat mat33 = q2.toRotMat33();
	cv::Mat mat44 = q2.toRotMat44();
	cout << "\nq2" << q2 << "	to mat33\n" <<  mat33 << endl;
	cout << "\nq2" << q2 << "	to mat44\n" <<  mat44 << endl;
	
	UnitQuaternion<double> q_from_mat(mat33);
	cout << "\nmat to q2" << q2 << endl;


	cout << "\n Quaternion inv:" << Quaternion<double>::dot(quat_q2, quat_q2) << endl;
	// cout << "\n UnitQuaternion inv:" <<Quaternion<double>:: dot(quat, q2) << endl;
	
	cout << q7.getAxis() << endl;
}	
