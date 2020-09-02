int test_queternion_funcs(){
	std::cout << "=================================" << std::endl;
	Quaternion<float> q1{2,1,3,4};
	q1 = q1.normalize();
	assert(q1.norm() == 1);
	bool ans = Quaternion<double>::isNormalized(cv::Vec<double, 4>{1,2,3,4});
	assert(ans == false);
	ans = Quaternion<double>::isNormalized(Quaternion<double>{1.1,2,3,4});
	assert(ans == false);
	ans = Quaternion<double>::isNormalized(Quaternion<double>{1,2,3,4} / sqrt(30));
	assert(ans == true);
	ans = Quaternion<double>::isNormalized(cv::Vec<double, 4>{1,2,3,4} / sqrt(30));
	assert(ans == true);
	double angle = 3.1415926 / 3;
	cv::Vec<double, 3> axis{1/sqrt(3),1/sqrt(3),1/sqrt(3)};	
	
	
	cv::Mat testMat = (cv::Mat_<double>(2,4) << 0,1,1,1,0,4,2,3);
	Quaternion<double>::transform(testMat, angle, axis);
	std::cout << "after transformation:\n" << testMat << std::endl;
	Quaternion<double> rotateQuat = Quaternion<double>::getRotQuat(angle, axis);
	Quaternion<double> betransed1(0,1,1,1);
	Quaternion<double> betransed2(0,4,2,3);
	
	std::cout << "One point[0,1,1,1]:" << rotateQuat * betransed1 * rotateQuat.conjugate()<<std::endl; 	
	std::cout << "One point[0,4,2,3]:" << rotateQuat * betransed2 * rotateQuat.conjugate()<<std::endl; 	
	
	// have some problem
	Quaternion<double> q(1,-4,-2,-3);
	q = q.normalize();
	std::cout << "\nQuat2mat: Quat:\n" << q <<"\nMat:\n" << q.toRotMat() << std::endl;
	testMat = q.toRotMat().colRange(1, 4).clone().rowRange(1, 4).clone();
	// testMat = (cv::Mat_<double>(3,3) << -39,12,24,12,-49,16,24,16,-25);
	Quaternion<double> q2(testMat);
	std::cout <<"\nMat2Quat：Mat:\n" << testMat << "\nQuat:" << q2 << std::endl;

	// test angle and axis calculation
	Quaternion<double> q3(Quaternion<double>::getRotQuat(angle, axis));
	std::cout << "\nOne Quaternion" << q3 << " is creat by \nangle:" << angle << "\naxis:" << axis << std::endl;
	std::cout << "after get Angle and getAxis: \nangle:" << q3.getAngle() << "\naxis:" << q3.getAxis() << std::endl;
	//assert(angle == q3.getAngle());  // have little difference
	axis = {0, 0, 1};
	double t = 0.5;
	Quaternion<double> q4(Quaternion<double>::getRotQuat(0, axis));
	Quaternion<double> q5(Quaternion<double>::getRotQuat(3.1415926, axis));
	Quaternion<double> q6(Quaternion<double>::slerp(q4, q5, t));
	std::cout << "\nslerp between q4: " << q4 << "q5: " << q5 << " with t=" << t << "\nq6:" << q6 << std::endl;
}
