#include <iostream>
#include "quaternion.cpp"
#include <gtest/gtest.h>
using namespace cv;
namespace opencv_test{ namespace {
class QuatTest: public ::testing::Test {
protected:
    void SetUp() override 
    {
        q1 = {1,2,3,4};
        q2 = {2.5,-2,3.5,4};
        q1Unit = {1 / sqrt(30), sqrt(2) /sqrt(15), sqrt(3) / sqrt(10), 2 * sqrt(2) / sqrt(15)};
        q4 = {5.6,2,3,4};
        q1Inv = {1.0 / 30, -1.0 / 15, -1.0 / 10, -2.0 / 15};
    }
    Quat<double> q1Inv;
    Quat<double> q1;
    Quat<double> q2;
    Quat<double> q1Unit;
    Quat<double> q4;
    double scalar = 2.5;

};


TEST_F(QuatTest, unaryOp){
    Quat<double> q1Conj{1,-2,-3,-4};

    EXPECT_EQ(q1.norm(), sqrt(30));
    EXPECT_EQ(q1.normalize(), q1Unit);
    EXPECT_EQ(q1.conjugate(), q1Conj);
    EXPECT_EQ(q1.inv(), q1Inv);
}

TEST_F(QuatTest, opeartor){
    Quat<double> minusQ{-1, -2, -3, -4};
    Quat<double> qAdd{3.5, 0, 6.5, 8};
    Quat<double> qMinus{-1.5, 4, -0.5, 0};
    Quat<double> qMultq{-20, 1, -5, 27};
    Quat<double> qMults{2.5, 5.0, 7.5, 10.0};
    Quat<double> qDvsq{1, 0, 0, 0};
    Quat<double> qDvss{1.0 / 2.5, 2.0 / 2.5, 3.0 / 2.5, 4.0 / 2.5};
    Quat<double> qOrigin(q1);
    
    EXPECT_EQ(-q1, minusQ);
    EXPECT_EQ(q1 + q2, qAdd);
    EXPECT_EQ(q1 - q2, qMinus);
    EXPECT_EQ(q1 * q2, qMultq);
    EXPECT_EQ(q1 * scalar, qMults);
    EXPECT_EQ(scalar * q1, qMults);
    EXPECT_EQ(q1 / q1, qDvsq);
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
}

TEST_F(QuatTest, quatAttrs){
    double angle = 2 * acos(1.0 / sqrt(30));
    Vec<double, 3> axis{sqrt(2) /sqrt(15), sqrt(3) / sqrt(10), 2 * sqrt(2) / sqrt(15)};
    Quat<double> qNull(1, 0, 0, 0);

    EXPECT_EQ(angle, q1.getAngle());
    EXPECT_EQ(angle, q1Unit.getAngle());
    EXPECT_ANY_THROW(qNull.getAngle());
    
    EXPECT_EQ(axis, q1.getAxis());
    //EXPECT_EQ(axis, q1Unit.getAxis());    
}

} // namespace 

}// opencv_test



int main(int argc, char **argv) {  
 std::cout << "Running main() from gtest_main.cc\n";  
 testing::InitGoogleTest(&argc, argv);  
 return RUN_ALL_TESTS();  
}
