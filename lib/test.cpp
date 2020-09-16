#include <iostream>
#include "quaternion.cpp"
#include <gtest/gtest.h>
/*
// Tests factorial of 0.

TEST(FactorialTest, HandlesZeroInput) {
  EXPECT_EQ(Factorial(0), 0) << "here is a problem";
}
TEST(FactorialTest, HandlesPositiveInput) {
  EXPECT_EQ(Factorial(1), 1);
  EXPECT_EQ(Factorial(2), 2);
  EXPECT_EQ(Factorial(3), 6);
  EXPECT_EQ(Factorial(8), 40320);
}
// gtest-main.cc  
*/
namespace opencv_test{ namespace {
class QuatTest: public ::testing::Test {
protected:
    void SetUp() override 
    {
        q1 = {1,2,3,4};
        q2 = {2.5,-2,3.5,4};
        q3 = {3.4,2,3,4};
        q4 = {5.6,2,3,4};
    }
    Quat<double> q1;
    Quat<double> q2;
    Quat<double> q3;
    Quat<double> q4;
};


TEST_F(QuatTest, unaryOp){
    Quat<double> q1Unit{1 / sqrt(30), sqrt(2) /sqrt(15), sqrt(3) / sqrt(10), 2 * sqrt(2) / sqrt(15)};
    Quat<double> q1Conj{1,-2,-3,-4};
    Quat<double> q1Inv(1.0 / 30, -1.0 / 15, -1.0 / 10, -2.0 / 15);
    EXPECT_EQ(q1.norm(), sqrt(30));
    EXPECT_EQ(q1.normalize(), q1Unit);
    EXPECT_EQ(q1.conjugate(), q1Conj);
    EXPECT_EQ(q1.inv(), q1Inv);
}

TEST_F(QuatTest, opeartor){
    Quat<double> qAdd{3.5, 0, 6.5, 8}
    Quat<double> qMinus{-1.5, 4, -0.5, 0}
    Quat<double> qMult{}
}

} // namespace 

}// opencv_test



int main(int argc, char **argv) {  
 std::cout << "Running main() from gtest_main.cc\n";  
 testing::InitGoogleTest(&argc, argv);  
 return RUN_ALL_TESTS();  
}
