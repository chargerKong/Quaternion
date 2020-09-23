#include <iostream>
#include "quaternion.cpp"
#include <iomanip>
#include <gtest/gtest.h>
//#include <opencv2/ts/coda_test.hpp>
using namespace cv;
namespace opencv_test{ namespace {
class QuatTest: public ::testing::Test {
protected:
    void SetUp() override 
    {
        q1 = {1,2,3,4};
        q2 = {2.5,-2,3.5,4};
        q1Unit = {1 / sqrt(30), sqrt(2) /sqrt(15), sqrt(3) / sqrt(10), 2 * sqrt(2) / sqrt(15)};
        q1Inv = {1.0 / 30, -1.0 / 15, -1.0 / 10, -2.0 / 15};
    }
    double scalar = 2.5;
    double angle = CV_PI;
    double qNorm2 = 2.5;
    Vec<double, 3> axis{1, 1, 1};
    Vec<double, 3> unAxis{0, 0, 0};
    Vec<double, 3> unitAxis{1.0 / sqrt(3), 1.0 / sqrt(3), 1.0 / sqrt(3)};
    Quat<double> q3{angle, axis};
    Quat<double> q3UnitAxis{angle, unitAxis};
    Quat<double> q3Norm2{angle, axis ,qNorm2};

    Quat<double> q1Inv;
    Quat<double> q1;
    Quat<double> q2;
    Quat<double> q1Unit;

    Quatd qNull{0, 0, 0, 0};
    Quatd qIdentity{1, 0, 0, 0};
};

TEST_F(QuatTest, constructor){
    Vec<double, 4> coeff{1, 2, 3, 4};
    EXPECT_EQ(Quat<double> (coeff), q1);
    EXPECT_EQ(q3, q3UnitAxis);
    EXPECT_ANY_THROW(Quat<double> q3(angle, unAxis));
    Mat R1 = (Mat_<double>(3, 3) << -1.0 / 3, 2.0 / 3 , 2.0 / 3,
                                   2.0 / 3 , -1.0 / 3, 2.0 / 3,
                                   2.0 / 3 , 2.0 / 3 , -1.0 / 3);
    Quat<double> qMat(R1);
    Mat R2 = (Mat_<double>(3, 3) << 0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0,
                                    1.0, 0.0, 0.0);
    Quatd qMat2(R2);
    EXPECT_EQ(qMat2, Quatd(0.5,0.5,0.5,0.5));
    EXPECT_EQ(qMat, q3);
    Vec3d rod{1, 2, 3};
    Quatd rodQuad{1.0 / sqrt(15), 1. / sqrt(15), 2. / sqrt(15), 3. / sqrt(15)};
    Quatd qRot(rod);
    EXPECT_EQ(qRot, rodQuad);
    EXPECT_NO_THROW(Vec3d(0, 0, 0));
}


TEST_F(QuatTest, basicfuns){
    Quat<double> q1Conj{1, -2, -3, -4};
    EXPECT_EQ(q3Norm2.normalize(), q3);
    EXPECT_EQ(q1.norm(), sqrt(30));
    EXPECT_EQ(q1.normalize(), q1Unit);
    EXPECT_ANY_THROW(qNull.normalize());
    EXPECT_EQ(q1.conjugate(), q1Conj);
    EXPECT_EQ(q1.inv(), q1Inv);
    EXPECT_EQ(inv(q1), q1Inv);
    EXPECT_EQ(q3.inv(true) * q3, qIdentity);
    EXPECT_EQ(q1.inv() * q1, qIdentity);
    EXPECT_ANY_THROW(inv(qNull));
    EXPECT_NO_THROW(q1.at(0));
    EXPECT_ANY_THROW(q1.at(4));
    Mat R = (Mat_<double>(3, 3) << -2.0 / 3, 2.0 / 15 , 11.0 / 15,
                                   2.0 / 3 , -1.0 / 3 , 2.0 / 3  ,
                                   1.0 / 3 , 14.0 / 15, -2.0 / 15);
    Mat q1RotMat = q1.toRotMat3x3();
    std::cout << q1RotMat << R << "\n";

    Mat q3RotMat3 = q3.toRotMat3x3();
    Mat q3RotMat4 = q3.toRotMat4x4();
    std::cout << q1.normalize().toRotMat3x3() << std::endl;
    std::cout << q1.normalize().toRotMat4x4() << std::endl;

    //ASSERT_MAT_NEAR(q1RotMat, R, 1e-6);
    /*
    Mat q1R4 = q1Unit.toRotMat4x4();
    Mat q1R3 = q1Unit.toRotMat3x3();
    Mat dp3 = (Mat_<double>(3, 1) << 1,2,3);
    Mat dp4 = (Mat_<double>(4, 1) << 0,1,2,3);
    std::cout << "dp4:" << q1R4 << "\n";
    std::cout << q1R3 * dp3 << "\n";
    std::cout << q1R4 * dp4 << "\n";
    */
    EXPECT_ANY_THROW(qNull.toRodrigues());
    Vec3d rodVec{q1[1] / q1[0], q1[2] / q1[0], q1[3] / q1[0]};
    Vec3d q1Rod = q1.toRodrigues();
    EXPECT_EQ(q1Rod[0], rodVec[0]);
    EXPECT_EQ(q1Rod[1], rodVec[1]);
    EXPECT_EQ(q1Rod[2], rodVec[2]);

    EXPECT_EQ(log(q1Unit, true), log(q1Unit));
    EXPECT_EQ(log(qIdentity, true), qNull);
    EXPECT_EQ(log(q3), Quatd(0, angle * unitAxis[0] / 2, angle * unitAxis[1] / 2, angle * unitAxis[2] / 2));
    EXPECT_ANY_THROW(log(qNull));
    EXPECT_EQ(log(Quatd(exp(1), 0, 0, 0)), qIdentity);

    EXPECT_EQ(exp(qIdentity), Quatd(exp(1), 0, 0, 0));
    EXPECT_EQ(exp(qNull), qIdentity);
    EXPECT_EQ(exp(Quatd(0, angle * unitAxis[0] / 2, angle * unitAxis[1] / 2, angle * unitAxis[2] / 2)), q3);

    EXPECT_EQ(power(q3, 2.0), Quatd(2*angle, axis));
    EXPECT_EQ(power(Quatd(0.5, 0.5, 0.5, 0.5), 2.0, true), Quatd(-0.5,0.5,0.5,0.5));
    EXPECT_EQ(power(Quatd(0.5, 0.5, 0.5, 0.5), -2.0), Quatd(-0.5,-0.5,-0.5,-0.5));
    EXPECT_EQ(sqrt(q1), power(q1, 0.5));
    EXPECT_EQ(exp(q3 * log(q1)), power(q1, q3));
    EXPECT_EQ(exp(q1 * log(q3)), power(q3, q1, true));
    EXPECT_EQ(crossProduct(q1, q3), (q1 * q3 - q3 * q1) / 2);
    EXPECT_EQ(sinh(qNull), qNull);
    EXPECT_EQ(sinh(q1), (exp(q1) - exp(-q1)) / 2);
    EXPECT_EQ(sinh(qIdentity), Quatd(sinh(1), 0, 0, 0));
    EXPECT_EQ(sinh(q1), Quatd(0.73233760604, -0.44820744998, -0.67231117497, -0.8964148999610843));
    EXPECT_EQ(cosh(qNull), qIdentity);
    EXPECT_EQ(cosh(q1), Quatd(0.961585117636, -0.34135217456, -0.51202826184, -0.682704349122));
    EXPECT_EQ(tanh(q1), sinh(q1) * inv(cosh(q1)));
    EXPECT_EQ(sin(qNull), qNull);
    EXPECT_EQ(sin(q1), Quatd(91.78371578403, 21.88648685303, 32.829730279543, 43.772973706058));
    EXPECT_EQ(cos(qNull), qIdentity);
    EXPECT_EQ(cos(q1), Quatd(58.9336461679, -34.0861836904, -51.12927553569, -68.17236738093));
    EXPECT_EQ(tan(q1), sin(q1)/cos(q1));
    EXPECT_EQ(sinh(asinh(q1)), q1);
    Quatd c1 = asinh(sinh(q1));
    EXPECT_EQ(sinh(c1), sinh(q1));
    EXPECT_EQ(cosh(acosh(q1)), q1);
    c1 = acosh(cosh(q1));
    EXPECT_EQ(cosh(c1), cosh(q1));
    EXPECT_EQ(tanh(atanh(q1)), q1);
    c1 = atanh(tanh(q1));
    EXPECT_EQ(tanh(q1), tanh(c1));
    EXPECT_EQ(asin(sin(q1)), q1);
    EXPECT_EQ(sin(asin(q1)), q1);
    EXPECT_EQ(acos(cos(q1)), q1);
    EXPECT_EQ(cos(acos(q1)), q1);
    EXPECT_EQ(atan(tan(q3)), q3);
    EXPECT_EQ(tan(atan(q1)), q1); // atan may not be calculate
}

TEST_F(QuatTest, opeartor){
    Quatd minusQ{-1, -2, -3, -4};
    Quatd qAdd{3.5, 0, 6.5, 8};
    Quatd qMinus{-1.5, 4, -0.5, 0};
    Quatd qMultq{-20, 1, -5, 27};
    Quatd qMults{2.5, 5.0, 7.5, 10.0};
    Quatd qDvss{1.0 / 2.5, 2.0 / 2.5, 3.0 / 2.5, 4.0 / 2.5};
    Quatd qOrigin(q1);
    
    EXPECT_EQ(-q1, minusQ);
    EXPECT_EQ(q1 + q2, qAdd);
    EXPECT_EQ(q1 - q2, qMinus);
    EXPECT_EQ(q1 * q2, qMultq);
    EXPECT_EQ(q1 * scalar, qMults);
    EXPECT_EQ(scalar * q1, qMults);
    EXPECT_EQ(q1 / q1, qIdentity);
    EXPECT_EQ(q1 / scalar, qDvss);
    q1 += q2;
    EXPECT_EQ(q1, qAdd);
    q1 -= q2;
    EXPECT_EQ(q1, qOrigin);
    q1 *= q2;
    EXPECT_EQ(q1, qMultq);
    q1 /= q2;
    EXPECT_EQ(q1, qOrigin);
    q1 *= scalar;
    EXPECT_EQ(q1, qMults);
    q1 /= scalar;
    EXPECT_EQ(q1, qOrigin);
    EXPECT_NO_THROW(q1[0]);
    EXPECT_NO_THROW(q1.at(0));
    EXPECT_ANY_THROW(q1[4]);
    EXPECT_ANY_THROW(q1.at(4));
}
 
TEST_F(QuatTest, quatAttrs){
    double angleQ1 = 2 * acos(1.0 / sqrt(30));
    Vec3d axis{0.3713906763541037, 0.557086014, 0.742781352 };
    Vec<double, 3> q1axis = q1.getAxis();
    
    EXPECT_EQ(angleQ1, q1.getAngle());
    EXPECT_EQ(angleQ1, q1Unit.getAngle());
    EXPECT_ANY_THROW(qIdentity.getAngle());
    EXPECT_ANY_THROW(qIdentity.getAxis());
    EXPECT_NEAR(axis[0], q1axis[0], 1e-6);
    EXPECT_NEAR(axis[1], q1axis[1], 1e-6);
    EXPECT_NEAR(axis[2], q1axis[2], 1e-6);
    EXPECT_NEAR(q3Norm2.norm(), qNorm2, 1e-6);
    EXPECT_EQ(q3Norm2.getAngle(), angle);
    EXPECT_NEAR(axis[0], axis[0], 1e-6);
    EXPECT_NEAR(axis[1], axis[1], 1e-6);
    EXPECT_NEAR(axis[2], axis[2], 1e-6);
    EXPECT_ANY_THROW(Quat<double>(angle, axis, 0));

}

TEST_F(QuatTest, interpolation){
    Quatd qNoRot(0, axis);
    Quatd qLerpInter(1.0 / 2, sqrt(3) / 6, sqrt(3) / 6, sqrt(3) / 6);
    EXPECT_EQ(Quatd::lerp(qNoRot, q3, 0), qNoRot);
    EXPECT_EQ(Quatd::lerp(qNoRot, q3, 1), q3);
    EXPECT_EQ(Quatd::lerp(qNoRot, q3, 0.5), qLerpInter);
    Quatd q3NrNn2(0, axis, qNorm2);
    EXPECT_EQ(Quatd::nlerp(q3NrNn2, q3Norm2, 0), qNoRot);
    EXPECT_EQ(Quatd::nlerp(q3NrNn2, q3Norm2, 1), q3);
    EXPECT_EQ(Quatd::nlerp(q3NrNn2, q3Norm2, 0.5), qLerpInter.normalize());
    EXPECT_EQ(Quatd::nlerp(qNoRot, q3, 0, true), qNoRot);
    EXPECT_EQ(Quatd::nlerp(qNoRot, q3, 1, true), q3);
    EXPECT_EQ(Quatd::nlerp(qNoRot, q3, 0.5, true), qLerpInter.normalize());
    Quatd q3Minus(-q3);
    EXPECT_EQ(Quatd::nlerp(qNoRot, q3, 0.4), -Quatd::nlerp(qNoRot, q3Minus, 0.4));
    EXPECT_EQ(Quatd::slerp(qNoRot, q3, 0, true), qNoRot);
    EXPECT_EQ(Quatd::slerp(qNoRot, q3, 1, true), q3);
    EXPECT_EQ(Quatd::slerp(qNoRot, q3, 0.5, true), -Quatd::nlerp(qNoRot, -q3, 0.5, true));
    EXPECT_EQ(Quatd::slerp(qNoRot, q1, 0.5), Quatd(0.76895194, 0.2374325, 0.35614876, 0.47486501));
    EXPECT_EQ(Quatd::slerp(-qNoRot, q1, 0.5), Quatd(0.76895194, 0.2374325, 0.35614876, 0.47486501));
    EXPECT_EQ(Quatd::slerp(qNoRot, -q1, 0.5), -Quatd::slerp(-qNoRot, q1, 0.5));

    Quat<double> tr1(0, axis);
	Quat<double> tr2(angle / 2, axis);
	Quat<double> tr3(angle, axis);
	Quat<double> tr4(angle, Vec3d{-1/sqrt(2),0,1/(sqrt(2))});
    EXPECT_ANY_THROW(Quatd::spline(qNull, tr1, tr2, tr3, 0));
    EXPECT_EQ(Quatd::spline(tr1, tr2, tr3, tr4, 0), tr2);
    EXPECT_EQ(Quatd::spline(tr1, tr2, tr3, tr4, 1), tr3);
    EXPECT_EQ(Quatd::spline(tr1, tr2, tr3, tr4, 0.6, true), Quatd::spline(tr1, tr2, tr3, tr4, 0.6));
    EXPECT_EQ(Quatd::spline(tr1, tr2, tr3, tr3, 0.5), Quatd::spline(tr1, -tr2, tr3, tr3, 0.5));
    EXPECT_EQ(Quatd::spline(tr1, tr2, tr3, tr3, 0.5), -Quatd::spline(-tr1, -tr2, -tr3, tr3, 0.5));
    EXPECT_EQ(Quatd::spline(tr1, tr2, tr3, tr3, 0.5), Quatd(0.336889853392, 0.543600719487, 0.543600719487, 0.543600719487));
    Quatd a = Quatd::spline(tr1, tr2, tr3, tr3, 0.5);

 /*   
    axis = {0,0,1};
    double angle = (CV_PI / 2);
    Quatd tt0(angle/2, axis);
    Quatd tt1(2*CV_PI - angle, axis);

    //Quat<double> tr1(0, axis);
	//Quat<double> tr2(angle / 2, axis);
	//Quat<double> tr3(angle, cv::Vec<double,3>{1/sqrt(3),1/sqrt(3),1/sqrt(3)});
	//Quat<double> tr4(angle, cv::Vec<double,3>{-1/sqrt(2),0,1/(sqrt(2))});
	Quat<double> tr5(angle, cv::Vec<double,3>{2/sqrt(5),0/sqrt(5),-1/(sqrt(5))});
	std::vector<Quat<double>> trs{-tr1,-tr2,-tr3,-tr4,-tr5};

    cv::Mat p = (cv::Mat_<double>(3,1) << 1, 0, 0);
	cv::Mat ans;
	Quat<double> t_12;

for (size_t i = 0; i < 1; ++i)
	{
        double j=0;
		while (j < 1)
		{
			Quat<double> q0, q1, q2, q3;
			q0 = trs[i];
			q1 = trs[i + 1];
			q2 = trs[i + 2];
			q3 = trs[i + 3];
				if(i == 2){
					q3 = trs[i+2];
				}
            //Quat<double> minu = -trs[i+1];
            //std::cout << minu << std::endl;
           // t_12 = Quat<double>::nlerp(trs[i], trs[i+1], j);
            //t_12 = Quat<double>::slerp(qNoRot, -q3, j);
			t_12 = Quat<double>::spline(-tr1,-tr2,tr3,-tr3,0.5,true);
            std::cout << t_12 << std::endl;
            j = j + 0.05;
			ans = (t_12.toRotMat3x3() * p).t();
			std::cout << ans << std::endl;
		}
	}	

 	        //t_12 = Quat<double>::slerp(trs[2],-trs[3],0.1);
			//ans = (t_12.toRotMat3x3() * p).t();
			//std::cout << ans;
*/
}
} // namespace 

}// opencv_test



int main(int argc, char **argv) {  
 std::cout << "Running main() from gtest_main.cc\n";  
 testing::InitGoogleTest(&argc, argv);  
 return RUN_ALL_TESTS();  
}
