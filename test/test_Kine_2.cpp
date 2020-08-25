// Copyright 2005, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// A sample program demonstrating using Google C++ testing framework.
//
// Author: wan@google.com (Zhanyong Wan)


// In this example, we use a more advanced feature of Google Test called
// test fixture.
//
// A test fixture is a place to hold objects and functions shared by
// all tests in a test case.  Using a test fixture avoids duplicating
// the test code necessary to initialize and cleanup those common
// objects for each test.  It is also useful for defining sub-routines
// that your tests need to invoke a lot.
//
// <TechnicalDetails>
//
// The tests share the test fixture in the sense of code sharing, not
// data sharing.  Each test is given its own fresh copy of the
// fixture.  You cannot expect the data modified by one test to be
// passed on to another test, which is a bad idea.
//
// The reason for this design is that tests should be independent and
// repeatable.  In particular, a test should not fail as the result of
// another test's failure.  If one test depends on info produced by
// another test, then the two tests should really be one big test.
//
// The macros for indicating the success/failure of a test
// (EXPECT_TRUE, FAIL, etc) need to know what the current test is
// (when Google Test prints the test result, it tells you which test
// each failure belongs to).  Technically, these macros invoke a
// member function of the Test class.  Therefore, you cannot use them
// in a global function.  That's why you should put test sub-routines
// in a test fixture.
//
// </TechnicalDetails>

#include "Observability.h"
#include "gtest/gtest.h"
//#include "gtest/gmock.h"

using namespace ORB_SLAM2;

namespace {
// To use a test fixture, derive a class from testing::Test.
class TestPredJacobian : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // initialize the obs class
        //        std::cout << "start setting up" << std::endl;

        // EuRoC cam param
        //        Camera2.f: 5.1369248
        //        Camera2.k1: -0.28340811
        //        Camera2.k2: 0.07395907
        //        Camera2.cx: 367.215
        //        Camera2.cy: 248.375
        //        Camera2.dx: 0.0112
        //        Camera2.dy: 0.01123325985
        //        Camera2.nCols: 752
        //        Camera2.nRows: 480

        // Matlab cam param
        double C2_f = 1.0;
        double C2_k1= 0;
        double C2_k2= 0;
        double C2_cx= 320;
        double C2_cy= 240;
        double C2_dx= 1.0/320.0;
        double C2_dy= 1.0/320.0;
        double C2_nRows= 480;
        double C2_nCols= 640;

        //        arma::mat C2_K;
        //        C2_K << C2_f/C2_dx << 0.0 << C2_cx << arma::endr
        //             << 0.0 << C2_f/C2_dy << C2_cy << arma::endr
        //             << 0.0 << 0.0 <<  1.0 << arma::endr;

        //        std::cout << "instantiate observability object" << std::endl;
        obs_ = new Observability(C2_f, C2_nRows, C2_nCols, C2_cx, C2_cy,
                                 C2_k1, C2_k2, C2_dx, C2_dy);

        //        std::cout << obs_->dx << "; " << obs_->dy << std::endl;
        //        std::cout << obs_->camera.dx << "; " << obs_->camera.dy << std::endl;

        //        std::cout << "fill in map and measurement variables" << std::endl;
        // fill in necessary members
        pFrame_ = new Frame();
        cv::Mat lmkMat(3, 1, CV_32F);
        MapPoint * oriMapPoints = new MapPoint[5];
        //
        lmkMat.at<float>(0,0) = -1.8360;
        lmkMat.at<float>(1,0) = -0.4160;
        lmkMat.at<float>(2,0) = 5.2535;
        oriMapPoints[0].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(130.8500,  219.6414, 1.0));
        //
        lmkMat.at<float>(0,0) = 1.5331;
        lmkMat.at<float>(1,0) = 2.4569;
        lmkMat.at<float>(2,0) = 6.6656;
        oriMapPoints[1].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(359.7823,  396.9450, 1.0));
        //
        lmkMat.at<float>(0,0) = -1.4521;
        lmkMat.at<float>(1,0) = -1.6277;
        lmkMat.at<float>(2,0) = 4.6702;
        oriMapPoints[2].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(144.9485,  109.6842, 1.0));
        //
        lmkMat.at<float>(0,0) = 1.8071;
        lmkMat.at<float>(1,0) = -0.6224;
        lmkMat.at<float>(2,0) = 3.6140;
        oriMapPoints[3].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(471.9332,  198.4909, 1.0));
        //
        lmkMat.at<float>(0,0) = 1.0833;
        lmkMat.at<float>(1,0) = -3.0235;
        lmkMat.at<float>(2,0) = 8.9990;
        oriMapPoints[4].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(333.1666,  135.6751, 1.0));
        //
        for (size_t i=0; i<5; ++i) {
            pFrame_->mvpMapPoints.push_back(&oriMapPoints[i]);
        }

        //        std::cout << "fill in pose variables" << std::endl;
        // 0.3000   -0.1000    1.0000    0.9992    0.0131    0.0314   -0.0209    3.0000   -1.0000   10.0000    0.2618    0.6283   -0.4189
        //        cv::Mat Rwc(3, 3, CV_32F);
        //        cv::Mat twc(3, 1, CV_32F);
        //        // translation
        //        twc.at<float>(0, 0) = 0.300;
        //        twc.at<float>(1, 0) = -0.1000;
        //        twc.at<float>(2, 0) = 1.0000;
        //        // quaternion
        //        arma::mat quat;
        //        quat << 0.9992 << 0.0131 << 0.0314 << -0.0209 << arma::endr;
        //        obs_->QUAT2DCM_float(quat.t(), Rwc);
        //        Twc << Rwc.at<float>(0,0) << Rwc.at<float>(0,1) << Rwc.at<float>(0,2) << twc.at<float>(0,0) << arma::endr
        //            << Rwc.at<float>(1,0) << Rwc.at<float>(1,1) << Rwc.at<float>(1,2) << twc.at<float>(1,0) << arma::endr
        //            << Rwc.at<float>(2,0) << Rwc.at<float>(2,1) << Rwc.at<float>(2,2) << twc.at<float>(2,0) << arma::endr
        //            << 0 << 0 << 0 << 1 << arma::endr;
        //        Tcw_prev = arma::eye<arma::mat>(4, 4);

        //        std::cout << "propagate pose param" << std::endl;
        obs_->Xv << 0.3000  << -0.1000 <<   1.0000  <<  0.9992  <<  0.0131  <<  0.0314 <<  -0.0209
                 <<  3.0000 <<  -1.0000 <<  10.0000  <<  0.2618  <<  0.6283 <<  -0.4189;
        obs_->predictPWLSVec(0.1, 3);

        //        std::cout << "fill in kinematic block for verification" << std::endl;
        arma::mat F_Q_tmp, F_Omg_tmp, F_Q_inSeg_tmp, F_Omg_inSeg_tmp;

        // segment 1
        //        print out the full-seg system matrix:
        F_Q_tmp << 0.9992 <<  -0.0131 << -0.0314  <<  0.0209 << arma::endr
                << 0.0131 <<   0.9992 <<  -0.0209 <<  -0.0314 << arma::endr
                << 0.0314  <<  0.0209  <<  0.9992  <<  0.0131 << arma::endr
                << -0.0209 <<   0.0314 <<  -0.0131  <<  0.9992 << arma::endr;

        F_Omg_tmp << -0.0013 <<  -0.0031 <<   0.0021 << arma::endr
                  << 0.0499  << 0.0010  <<  0.0016 << arma::endr
                  << -0.0011  <<  0.0499  << -0.0006 << arma::endr
                  << -0.0016  <<  0.0007  <<  0.0499 << arma::endr;

        //        print out the in-seg system matrix:
        F_Q_inSeg_tmp << 1.0000 <<  -0.0010 <<  -0.0024  <<  0.0016 << arma::endr
                      << 0.0010 <<   1.0000 <<  -0.0016  << -0.0024 << arma::endr
                      << 0.0024  <<  0.0016  <<  1.0000  <<  0.0010 << arma::endr
                      << -0.0016 <<   0.0024 <<  -0.0010 <<   1.0000 << arma::endr;

        F_Omg_inSeg_tmp << -0.0001 <<  -0.0001 <<   0.0001 << arma::endr
                        << 0.0038  <<  0.0001  <<  0.0001 << arma::endr
                        << -0.0001  <<  0.0038  << -0.0001 << arma::endr
                        << -0.0001  <<  0.0001  <<  0.0038 << arma::endr;
        //
        F_Q.push_back(F_Q_tmp);
        F_Omg.push_back(F_Omg_tmp);
        F_Q_inSeg.push_back(F_Q_inSeg_tmp);
        F_Omg_inSeg.push_back(F_Omg_inSeg_tmp);

        // segment 2
        //        print out the full-seg system matrix:
        F_Q_tmp << 0.9992  << -0.0131 <<  -0.0314  <<  0.0209 << arma::endr
                <<  0.0131  <<  0.9992 <<  -0.0209 <<  -0.0314 << arma::endr
                 <<  0.0314  <<  0.0209 <<   0.9992 <<  0.0131 << arma::endr
                  << -0.0209  <<  0.0314  << -0.0131  <<  0.9992 << arma::endr;

        F_Omg_tmp <<    -0.0020  << -0.0047 <<   0.0031 << arma::endr
                     <<     0.0498  <<  0.0020  <<  0.0032 << arma::endr
                         <<   -0.0021  <<  0.0497  << -0.0012 << arma::endr
                           <<  -0.0031  <<  0.0014 <<   0.0498 << arma::endr;

        //        print out the in-seg system matrix:
        F_Q_inSeg_tmp << 1.0000 <<  -0.0010  << -0.0024  <<  0.0016 << arma::endr
                      <<  0.0010  <<  1.0000  << -0.0016 <<  -0.0024 << arma::endr
                       <<  0.0024  <<  0.0016  <<  1.0000  <<  0.0010 << arma::endr
                        << -0.0016  <<  0.0024  << -0.0010 <<  1.0000 << arma::endr;

        F_Omg_inSeg_tmp << -0.0001  << -0.0003  <<  0.0002 << arma::endr
                        <<  0.0038 <<   0.0002  <<  0.0002 << arma::endr
                         << -0.0002 <<   0.0038 <<  -0.0001 << arma::endr
                         << -0.0002 <<   0.0001  <<  0.0038 << arma::endr;
        //
        F_Q.push_back(F_Q_tmp);
        F_Omg.push_back(F_Omg_tmp);
        F_Q_inSeg.push_back(F_Q_inSeg_tmp);
        F_Omg_inSeg.push_back(F_Omg_inSeg_tmp);

        // segment 3
        //        print out the full-seg system matrix:
        F_Q_tmp << 0.9992 <<  -0.0131  << -0.0314  <<  0.0209 << arma::endr
                <<  0.0131  <<  0.9992 <<  -0.0209 <<  -0.0314 << arma::endr
                 <<  0.0314 <<   0.0209  <<  0.9992  <<  0.0131 << arma::endr
                  << -0.0209  <<  0.0314 <<  -0.0131 <<   0.9992 << arma::endr;

        F_Omg_tmp << -0.0026 <<  -0.0063  <<  0.0042 << arma::endr
                  <<  0.0496  <<  0.0031   << 0.0047 << arma::endr
                   <<  -0.0032 <<   0.0495  << -0.0018 << arma::endr
                    <<  -0.0047 <<   0.0021 <<   0.0496 << arma::endr;

        //        print out the in-seg system matrix:
        F_Q_inSeg_tmp << 1.0000 <<  -0.0010 <<  -0.0024   << 0.0016 << arma::endr
                      <<   0.0010 <<   1.0000  << -0.0016  << -0.0024 << arma::endr
                        <<  0.0024 <<   0.0016 <<   1.0000  <<  0.0010 << arma::endr
                         <<  -0.0016  <<  0.0024 <<  -0.0010 <<   1.0000 << arma::endr;

        F_Omg_inSeg_tmp << -0.0002  << -0.0004 <<   0.0002 << arma::endr
                        <<   0.0038 <<   0.0002  <<  0.0004 << arma::endr
                          <<  -0.0002 <<   0.0038  << -0.0002 << arma::endr
                           <<  -0.0004  <<  0.0002 <<   0.0038 << arma::endr;
        //
        F_Q.push_back(F_Q_tmp);
        F_Omg.push_back(F_Omg_tmp);
        F_Q_inSeg.push_back(F_Q_inSeg_tmp);
        F_Omg_inSeg.push_back(F_Omg_inSeg_tmp);

    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    Observability * obs_;
    Frame * pFrame_;
    //    arma::mat Twc, Tcw_prev;
    vector<arma::mat> F_Q, F_Omg, F_Q_inSeg, F_Omg_inSeg;
};

// When you have a test fixture, you define a test using TEST_F
// instead of TEST.
//TEST_F(TestObsJacobian, MacroDefinition) {
//    EXPECT_EQ(OBS_SEGMENT_NUM, 3);
//}

//// Tests the Measurement Jacobian.
//TEST_F(TestObsJacobian, Measurement) {
//}

// Tests the Kinematic/System Jacobian.
TEST_F(TestPredJacobian, PredKinematic) {
    //
    //    std::cout << "actual varify start" << std::endl;
    for (size_t i=0; i<obs_->kinematic.size(); ++i) {

        //        std::cout << obs_->kinematic[i].Xv << std::endl;
        //        std::cout << obs_->kinematic[i].Tcw << std::endl;

        //        std::cout << obs_->kinematic[i].F_Q  << std::endl;
        //        std::cout << obs_->kinematic[i].F_Omg  << std::endl;
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Q - F_Q[i], "inf"), 0, 0.002);
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Omg - F_Omg[i], "inf"), 0, 0.0002);
        //
        //        std::cout << obs_->kinematic[i].F_Q_inSeg  << std::endl;
        //        std::cout << obs_->kinematic[i].F_Omg_inSeg  << std::endl;
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Q_inSeg - F_Q_inSeg[i], "inf"), 0, 0.0002);
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Omg_inSeg - F_Omg_inSeg[i], "inf"), 0, 0.0002);
    }
}

// Tests the projection func
TEST_F(TestPredJacobian, CamProjection) {
    //
    //    std::cout << "actual varify start" << std::endl;

    //    std::cout << "testing " << pFrame_->mvpMapPoints.size() << " lmks:" << std::endl;

    for (size_t j=0; j<pFrame_->mvpMapPoints.size(); ++j) {

        //
        //        std::cout << "-------------" << std::endl;
        //        std::cout << "measured lmk " << j << ": " << pFrame_->mvKeysUn[j].pt.x << ", " << pFrame_->mvKeysUn[j].pt.y << std::endl;

        for (size_t i=0; i<2; ++i) {

            cv::Mat lPt = pFrame_->mvpMapPoints[j]->GetWorldPos();
            //            std::cout << "lmk world coord " << lPt << std::endl;
            //            std::cout << obs_->kinematic[i].Tcw << std::endl;

            float u, v;
            arma::mat proj_jacob;
            obs_->project_Point_To_Frame(lPt, obs_->kinematic[i].Tcw, u, v, proj_jacob);

            //            std::cout << "predicted lmk " << j << " at segment " << i << std::endl;
            //            std::cout << u << ", " << v << std::endl;

            if (i == 0) {
                EXPECT_NEAR(u, pFrame_->mvKeysUn[j].pt.x, 10.0);
                EXPECT_NEAR(v, pFrame_->mvKeysUn[j].pt.y, 10.0);
            }
        }
    }
}

}  // namespace
