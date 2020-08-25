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
class TestMeasJacobian : public testing::Test {
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

        //        std::cout << "fill in map and measurement variables" << std::endl;
        // fill in necessary members
        pFrame_ = new Frame();
        cv::Mat lmkMat(3, 1, CV_32F);
        MapPoint * oriMapPoints = new MapPoint[5];
        arma::mat H13, H47;
        //
        lmkMat.at<float>(0,0) = -0.7118;
        lmkMat.at<float>(1,0) = 1.2178;
        lmkMat.at<float>(2,0) = 3.2776;
        oriMapPoints[0].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(135.0290,  439.5857, 1.0));

        H13 << -151.9172 << 8.2869 << -72.2803 << arma::endr
            << -0.6774 << -149.5428 << 86.2222 << arma::endr;

        H47 << 37.3479 << -222.5649 << -837.3745 << 387.1184 << arma::endr
            << -10.8979 << 894.5779 << 194.2587 << 330.4412 << arma::endr;

        H_13_.push_back(H13);
        H_47_.push_back(H47);

        //
        lmkMat.at<float>(0,0) =  -0.3000;
        lmkMat.at<float>(1,0) = 0.0641;
        lmkMat.at<float>(2,0) = 3.1522;
        oriMapPoints[1].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(199.2914,  274.5783, 1.0));

        H13 << -154.8918  <<  7.7050 << -43.7711 << arma::endr
            << -5.6234 << -152.1902  << 10.0400 << arma::endr;

        H47 << 24.0824 << -33.7441 << -719.1049 <<  49.4903 << arma::endr
            << -4.3712 << 652.3041  <<  4.0526 << 205.1686 << arma::endr;

        H_13_.push_back(H13);
        H_47_.push_back(H47);

        //
        lmkMat.at<float>(0,0) = 0.5960;
        lmkMat.at<float>(1,0) = -0.4677;
        lmkMat.at<float>(2,0) = 2.9845;
        oriMapPoints[2].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(339.6502,  197.7743, 1.0));

        H13  <<  -157.9638  <<  6.1337 <<  24.6956 << arma::endr
              << -8.3047 << -158.4693 << -28.1207 << arma::endr;

        H47 <<  18.3506 << -25.5025 << -643.4293 << -105.3739 << arma::endr
             << -10.0812 << 652.6057 <<  -4.0073 << -79.2158 << arma::endr;

        H_13_.push_back(H13);
        H_47_.push_back(H47);

        //
        lmkMat.at<float>(0,0) = -1.4897;
        lmkMat.at<float>(1,0) = -1.3683;
        lmkMat.at<float>(2,0) = 3.2295;
        oriMapPoints[3].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(34.4250,   55.4036, 1.0));

        H13 << -156.6515 <<   9.6818 << -120.2429 << arma::endr
            << -11.9145 << -146.3540 << -92.8235 << arma::endr;

        H47 <<  0.0230  <<  0.2988  << -1.1282 <<  -0.4087 << arma::endr
             << 0.0108  <<  0.8799 <<  -0.3600  <<  0.5262 << arma::endr;
        H47 = H47 * 1000;

        H_13_.push_back(H13);
        H_47_.push_back(H47);

        //
        lmkMat.at<float>(0,0) = 0.4827;
        lmkMat.at<float>(1,0) = 3.2980;
        lmkMat.at<float>(2,0) = 5.8807;
        oriMapPoints[4].SetWorldPos(lmkMat);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(298.2795,  477.0540, 1.0));

        H13 << -66.8818 <<   2.8440   << 0.5241 << arma::endr
            << 0.2084 << -68.1106 <<  47.4118 << arma::endr;

        H47 << 30.3666 << -24.8150 << -647.0808 << 462.9968 << arma::endr
            << -12.9879 << 987.4336 <<   5.0419  <<  4.9136 << arma::endr;

        H_13_.push_back(H13);
        H_47_.push_back(H47);


        //
        for (size_t i=0; i<5; ++i) {
            pFrame_->mvpMapPoints.push_back(&oriMapPoints[i]);
        }

//                std::cout << "propagate pose param" << std::endl;
        obs_->Xv << 0.3000  << -0.1000 <<   1.0000  <<  0.9992  <<  0.0131  <<  0.0314 <<  -0.0209
                 <<  3.0000 <<  -1.0000 <<  10.0000  <<  0.2618  <<  0.6283 <<  -0.4189;
        obs_->predictPWLSVec(0.1, 1);
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
    vector<arma::mat> H_13_, H_47_;
};

// When you have a test fixture, you define a test using TEST_F
// instead of TEST.
//TEST_F(TestObsJacobian, MacroDefinition) {
//    EXPECT_EQ(OBS_SEGMENT_NUM, 3);
//}

// Tests the Measurement Jacobian.
TEST_F(TestMeasJacobian, Measurement) {
    //
    //    std::cout << "testing " << pFrame_->mvpMapPoints.size() << " lmks:" << std::endl;

    for (size_t j=0; j<pFrame_->mvpMapPoints.size(); ++j) {

        //
//                std::cout << "-------------" << std::endl;
//                std::cout << "measured lmk " << j << ": " << pFrame_->mvKeysUn[j].pt.x << ", " << pFrame_->mvKeysUn[j].pt.y << std::endl;

        cv::Mat lPt = pFrame_->mvpMapPoints[j]->GetWorldPos();
        //            std::cout << "lmk world coord " << lPt << std::endl;
        //            std::cout << obs_->kinematic[i].Tcw << std::endl;

        float u, v;
        arma::mat proj_jacob;
        obs_->project_Point_To_Frame(lPt, obs_->kinematic[0].Tcw, u, v, proj_jacob);

        //        std::cout << "predicted lmk " << j << std::endl;
        //        std::cout << u << ", " << v << std::endl;

        EXPECT_NEAR(u, pFrame_->mvKeysUn[j].pt.x, 10.0);
        EXPECT_NEAR(v, pFrame_->mvKeysUn[j].pt.y, 10.0);

        //        std::cout << "estimated meas. Jacobian block: " << std::endl;
        arma::mat H13, H47, H_proj;
        // Feature position
        arma::rowvec Y = arma::zeros<arma::rowvec>(3);
        Y[0] = lPt.at<float>(0);
        Y[1] = lPt.at<float>(1);
        Y[2] = lPt.at<float>(2);
        // Measurement
        arma::rowvec Z = arma::zeros<arma::rowvec>(2);
        Z[0] = pFrame_->mvKeysUn[j].pt.x;
        Z[1] = pFrame_->mvKeysUn[j].pt.y;

        float res_u, res_v;
        obs_->compute_H_subblock_complete(obs_->kinematic[0].Xv, Y, Z, H13, H47, H_proj, res_u, res_v);
        EXPECT_NEAR(res_u, 0.0, 10.0);
        EXPECT_NEAR(res_v, 0.0, 10.0);

        //        std::cout << "H13 = " << H13 << std::endl;
        //        std::cout << "H47 = " << H47 << std::endl;

        //        std::cout << "actual meas. Jacobian block: " << std::endl;
        //        std::cout << "H13 = " << H_13_[j] << std::endl;
        //        std::cout << "H47 = " << H_47_[j] << std::endl;

        EXPECT_NEAR(arma::norm(H_13_[j] - H13, "inf"), 0, 0.25);
        EXPECT_NEAR(arma::norm(H_47_[j] - H47, "inf"), 0, 0.25);
        //

    }
}

}  // namespace
