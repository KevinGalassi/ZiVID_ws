//TO TEST PSEUDE INVERSE MATRIXES
/* Tests should be implemented for these functions:
-weighted_normalized_pinv()
-generalized_pinv()
-control()
-t_interpolation()
-update_Jk()
-update_A()
-update_Q()
-QuatInterp()
-round2()
-gbellmf()
-frf()
-sigmoid()
-splitD()

*/
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>

#include "move_rt/functions.hpp"
#include "gtest/gtest.h"
#include "ros/ros.h"

using namespace std;


class TestInverseKinematics: public testing::Test
{
    protected:

       Eigen::Matrix<Eigen::Matrix<long double,Eigen::Dynamic,Eigen::Dynamic>,3,1> JkQk_1;
       Eigen::Matrix<Eigen::Matrix<long double,Eigen::Dynamic,Eigen::Dynamic>,3,1> A;
       Eigen::Matrix<Eigen::Matrix<long double,Eigen::Dynamic,Eigen::Dynamic>,3,1> Q;
       long double eta {0.001};

        void SetUp()
        {

            JkQk_1(0,0).resize(6,6);
            JkQk_1(1,0).resize(6,6);
            JkQk_1(2,0).resize(6,6);

            JkQk_1(0,0)<<0.833333, -0.166667, -0.166667, -0.166667, -0.166667, -0.166667,
                        -0.166667,  0.833333, -0.166667, -0.166667, -0.166667, -0.166667,
                        -0.166666, -0.166666,  0.833334, -0.166666, -0.166666, -0.166666,
                        -0.166667, -0.166667, -0.166667,  0.833333, -0.166667, -0.166667,
                        -0.166667, -0.166667, -0.166667, -0.166667,  0.833333, -0.166667,
                        -0.166666, -0.166666, -0.166666, -0.166666, -0.166666,  0.833334;

            JkQk_1(1,0)<<0.833333, -0.166667, -0.166667, -0.166667, -0.166667, -0.166667,
                        -0.166667,  0.833333, -0.166667, -0.166667, -0.166667, -0.166667,
                        -0.166666, -0.166666,  0.833334, -0.166666, -0.166666, -0.166666,
                        -0.166667, -0.166667, -0.166667,  0.833333, -0.166667, -0.166667,
                        -0.166667, -0.166667, -0.166667, -0.166667,  0.833333, -0.166667,
                        -0.166666, -0.166666, -0.166666, -0.166666, -0.166666,  0.833334;

            JkQk_1(2,0)<<1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1;


            Q(0,0).resize(6,6);
            Q(1,0).resize(6,6);
            Q(2,0).resize(6,6);

            Q(0,0)<<1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0,
                        0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 1;
            Q(1,0)<<0.833333, -0.166667, -0.166667, -0.166667, -0.166667, -0.166667,
                    -0.166667,  0.833333, -0.166667, -0.166667, -0.166667, -0.166667,
                    -0.166666, -0.166666,  0.833334, -0.166666, -0.166666, -0.166666,
                    -0.166667, -0.166667, -0.166667,  0.833333, -0.166667, -0.166667,
                    -0.166667, -0.166667, -0.166667, -0.166667,  0.833333, -0.166667,
                    -0.166666, -0.166666, -0.166666, -0.166666, -0.166666,  0.833334;

            Q(2,0)<<1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;



            A(0,0).resize(6,6);
            A(1,0).resize(6,6);
            A(2,0).resize(6,6);


            A(0,0)<<1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

            A(1,0)<<1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

            A(2,0)<<1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        }


        void TearDown()
        {

        }
};

TEST_F(TestInverseKinematics, weightedPseudoInverse)
{
    Functions  myfunc;
    long double proximity {1e-3};
    Eigen::Matrix<Eigen::Matrix<long double, Eigen::Dynamic,Eigen::Dynamic>,3,1> weighted_normalized_pinv;

    weighted_normalized_pinv(0,0).resize(6,6);
    weighted_normalized_pinv(1,0).resize(6,6);
    weighted_normalized_pinv(2,0).resize(6,6);

    weighted_normalized_pinv(0,0)<<0.833333, -0.166666, -0.166667, -0.166666, -0.166667, -0.166666,
            -0.166666,  0.833333, -0.166667, -0.166666, -0.166667, -0.166666,
            -0.166667, -0.166667,  0.833333, -0.166667, -0.166667, -0.166667,
            -0.166667, -0.166667, -0.166667,  0.833334, -0.166666, -0.166667,
            -0.166667, -0.166667, -0.166667, -0.166666,  0.833333, -0.166667,
            -0.166666, -0.166666, -0.166667, -0.166667, -0.166667,  0.833334;
    weighted_normalized_pinv(1,0)<<0.83333, -0.166669, -0.166667, -0.166669, -0.166672, -0.166443,
            -0.166669,   0.83333, -0.166667, -0.166669, -0.166672, -0.166443,
            -0.16667,  -0.16667,  0.833332, -0.166669, -0.166671, -0.166443,
            -0.16667,  -0.16667, -0.166667,  0.833332, -0.166672, -0.166443,
            -0.16667,  -0.16667, -0.166667, -0.166669,  0.833329, -0.166443,
            -0.166669, -0.166669, -0.166667, -0.166669, -0.166671,  0.833556;
    weighted_normalized_pinv(2,0)<<0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778,
            0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778,
            0.0277777, 0.0277777, 0.0277777, 0.0277777, 0.0277777, 0.0277777,
            0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778,
            0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778, 0.0277778,
            0.0277777, 0.0277777, 0.0277777, 0.0277777, 0.0277777, 0.0277777;
    EXPECT_TRUE(myfunc.weighted_normalized_pinv(JkQk_1(0,0),A(0,0),Q(0,0),eta).isApprox(weighted_normalized_pinv(0,0),proximity));
    EXPECT_TRUE(myfunc.weighted_normalized_pinv(JkQk_1(1,0),A(1,0),Q(1,0),eta).isApprox(weighted_normalized_pinv(1,0),proximity));
    EXPECT_TRUE(myfunc.weighted_normalized_pinv(JkQk_1(2,0),A(2,0),Q(2,0),eta).isApprox(weighted_normalized_pinv(2,0),proximity));
}

class TestTinterpolation: public testing::Test
{
    Eigen::Matrix<long double, Eigen::Dynamic, 1> vec;
    Eigen::Matrix<long double, Eigen::Dynamic, 1> xc;
    double t;
    void SetUp()
    {

    }
    void TearDown()
    {

    }
};

TEST_F(TestTinterpolation, testTinterpolation)
{

}

TEST(TestSimpleMathFunc, gbellmfTest)
{
    Functions myfunc;
    long double offset = 1e-4;
    std::vector<long double> x;
    std::vector<long double> y;
    x.push_back(1e+06);
    x.push_back(1000.16);
    x.push_back(6.63486e-08);
    x.push_back(414509);
    x.push_back(4.43216e+06);
    x.push_back(0.0219777);

    y.push_back(4e-14);
    y.push_back(3.99874e-08);
    y.push_back(1);
    y.push_back(2.32805e-13);
    y.push_back(2.03624e-15);
    y.push_back(0.988069);

    long double a {0.2};
    long double b {1.0};
    long double c {0.0};
    try
    {
        for(int i : y)
        EXPECT_NEAR(y[i], gbellmf(x[i],a,b,c),offset);
        std::cout<<"GbellMF testing was implemented"<<'\n';
    }
    catch(...)
    {
       ADD_FAILURE()<<"Uncaught Exception";
    }


}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"UnitTEST");
    ros::NodeHandle node;
    return RUN_ALL_TESTS();
}
