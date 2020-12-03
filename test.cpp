// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

//#include "test_precomp.hpp"
#include "dualquaternion.hpp"
#include <gtest/gtest.h>
//#include <opencv2/ts/cuda_test.hpp>
using namespace cv;
namespace opencv_test{ namespace {
class DualQuatTest: public ::testing::Test {
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
    int qNorm2 = 2;
    Vec<double, 3> axis{1, 1, 1};
    Vec<double, 3> unAxis{0, 0, 0};
    Vec<double, 3> unitAxis{1.0 / sqrt(3), 1.0 / sqrt(3), 1.0 / sqrt(3)};

    Quat<double> q1Inv;
    Quat<double> q1;
    Quat<double> q2;
    Quat<double> q1Unit;

    Quatd qNull{0, 0, 0, 0};
    Quatd qIdentity{1, 0, 0, 0};
    DualQuatd dq1{1, 2, 3, 4, 5, 6, 7, 8};
    Quatd trans{0, 0, 0, 5};
    double rotation_angle = 2 * CV_PI / 3;
    DualQuatd dq2 = DualQuatd::createFromAngleAxisTrans(rotation_angle, axis, trans);
    DualQuatd dqAllOne{1, 1, 1, 1, 1, 1, 1, 1};
    DualQuatd dqAllZero{0, 0, 0, 0, 0, 0, 0, 0};
    DualQuatd dqIdentity{1, 0, 0, 0, 0, 0, 0, 0};
    DualQuatd dqTrans{1, 0, 0, 0, 0, 2, 3, 4};
    DualQuatd dqOnlyTrans{0, 0, 0, 0, 0, 2, 3, 4};
    DualQuatd dualNumber1{-3,0,0,0,-31.1,0,0,0};
    DualQuatd dualNumber2{4,0,0,0,5.1,0,0,0};
};

TEST_F(DualQuatTest, constructor){

    EXPECT_EQ(dq1, DualQuatd::createFromQuat(Quatd(1, 2, 3, 4), Quatd(5, 6, 7, 8)));
    EXPECT_EQ(dq2 * dq2.conjugate(), dqIdentity);
    EXPECT_NEAR(dq2.getRotation(QUAT_ASSUME_UNIT).norm(), 1, 1e-6);
    EXPECT_NEAR(dq2.getRealQuat().dot(dq2.getDualQuat()), 0, 1e-6);
    EXPECT_EQ(dq2.getTranslation(QUAT_ASSUME_UNIT), trans);
    DualQuatd q_conj = DualQuatd::createFromQuat(dq2.getRealQuat().conjugate(), -dq2.getDualQuat().conjugate());
    DualQuatd q{1,0,0,0,0,3,0,0};
    EXPECT_EQ(dq2 * q * q_conj, DualQuatd(1,0,0,0,0,0,3,5));
    Matx44d R1 = dq2.toMat();
    DualQuatd dq3 = DualQuatd::createFromMat(R1);
    EXPECT_EQ(dq3, dq2);
}
TEST_F(DualQuatTest, operator){
    EXPECT_EQ(dq1 - dqAllOne, DualQuatd(0, 1, 2, 3, 4, 5, 6, 7));
    EXPECT_EQ(-dq1, DualQuatd(-1, -2, -3, -4, -5, -6, -7, -8));
    EXPECT_EQ(dq1 + dqAllOne, DualQuatd(2, 3, 4, 5, 6, 7, 8, 9));
    EXPECT_EQ(dq1 / dq1, dqIdentity);
    DualQuatd dq12{2, 4, 6, 8, 10, 12, 14, 16};
    EXPECT_EQ(dq1 * 2.0, dq12);
}

TEST_F(DualQuatTest, basic_ops){
    EXPECT_EQ(dq1.getRealQuat(), Quatd(1, 2, 3, 4));
    EXPECT_EQ(dq1.getDualQuat(), Quatd(5, 6, 7, 8));
    EXPECT_EQ(dq1.conjugate(), DualQuatd::createFromQuat(dq1.getRealQuat().conjugate(), dq1.getDualQuat().conjugate()));
    EXPECT_EQ((dq2 * dq1).conjugate(), dq1.conjugate() * dq2.conjugate());
    EXPECT_EQ(dq1.conjugate() * dq1, dq1.norm() * dq1.norm());
    EXPECT_EQ(dq1.conjugate() * dq1, dq1.norm().power(2.0));
    EXPECT_EQ(dualNumber2.power(2.0), DualQuatd(16, 0, 0, 0, 40.8, 0, 0, 0));
    EXPECT_EQ(dq1.power(2.0), (2.0 * dq1.log()).exp());
    EXPECT_EQ(dq2.power(3.0 / 2, QUAT_ASSUME_UNIT).power(4.0 / 3, QUAT_ASSUME_UNIT), dq2 * dq2);
    EXPECT_EQ(dq2.power(-0.5).power(2.0), dq2.inv());
    EXPECT_EQ((dq2.norm() * dq1).power(2.0), dq1.power(2.0) * dq2.norm().power(2.0));

    DualQuatd q1norm = dq1.normalize();
    EXPECT_EQ(dq2.norm(), dqIdentity);
    EXPECT_NEAR(q1norm.getRealQuat().norm(), 1, 1e-6);
    EXPECT_NEAR(q1norm.getRealQuat().dot(q1norm.getDualQuat()), 0, 1e-6);
    EXPECT_NEAR(dq1.getRotation().norm(), 1, 1e-6);
    EXPECT_NEAR(dq1.getTranslation().w, 0, 1e-6);
    EXPECT_EQ(dq1.inv() * dq1, dqIdentity);
    EXPECT_EQ(dq2.inv(QUAT_ASSUME_UNIT) * dq2, dqIdentity);
    EXPECT_EQ(dq2.inv(), dq2.conjugate());
    EXPECT_EQ(dqIdentity.inv(), dqIdentity);
    EXPECT_ANY_THROW(dqAllZero.inv());
    DualQuatd dq = DualQuatd::createFromAngleAxisTrans(8*CV_PI/5, Vec3d{0,0,1}, Quatd{0,0,0,10});
    EXPECT_EQ(DualQuatd::sclerp(dqIdentity, dq, 0.5), DualQuatd::sclerp(-dqIdentity, dq, 0.5, false));
    EXPECT_EQ(DualQuatd::sclerp(dqIdentity, dq, 0), -dqIdentity);
    EXPECT_EQ(DualQuatd::sclerp(dqIdentity, dq2, 1), dq2);
    EXPECT_EQ(DualQuatd::sclerp(dqIdentity, dq2, 0.4, false, QUAT_ASSUME_UNIT), DualQuatd(0.91354546, 0.23482951, 0.23482951, 0.23482951, -0.23482951, -0.47824988, 0.69589767, 0.69589767));
    EXPECT_EQ(dqAllZero.exp(), dqIdentity);
    EXPECT_EQ(dqIdentity.log(), dqAllZero);
    EXPECT_EQ(dualNumber1 * dualNumber2, dualNumber2 * dualNumber1);
    EXPECT_EQ(dualNumber2.exp().log(), dualNumber2);
    EXPECT_EQ(dq2.log(QUAT_ASSUME_UNIT).exp(), dq2);
    EXPECT_EQ(dqIdentity.log(QUAT_ASSUME_UNIT).exp(), dqIdentity);
    EXPECT_EQ(dq1.log().exp(), dq1);
    EXPECT_EQ(dqTrans.log().exp(), dqTrans);

    Matx44d R1 = dq2.toMat();
    Matx41d point{
        3, 0, 0, 1
    };
    //EXPECT_MAT_NEAR(R1 * point,  Matx41d{0, 3, 5, 1});

}

} // namespace

}// opencv_test

int main(int argc, char **argv) {
 std::cout << "Running main() from gtest_main.cc\n";
 testing::InitGoogleTest(&argc, argv);
 return RUN_ALL_TESTS();
}
