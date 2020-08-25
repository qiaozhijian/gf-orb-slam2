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
class TestInstJacobian : public testing::Test {
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
        double C2_dx= 320;
        double C2_dy= 320;
        double C2_nRows= 480;
        double C2_nCols= 640;

        //        arma::mat C2_K;
        //        C2_K << C2_f/C2_dx << 0.0 << C2_cx << arma::endr
        //             << 0.0 << C2_f/C2_dy << C2_cy << arma::endr
        //             << 0.0 << 0.0 <<  1.0 << arma::endr;

//                std::cout << "instantiate observability object" << std::endl;
        obs_ = new Observability(C2_f, C2_nRows, C2_nCols, C2_cx, C2_cy,
                                 C2_k1, C2_k2, C2_dx, C2_dy);

        //        std::cout << "fill in pose variables" << std::endl;
        //0.0300   -0.0100    0.1000    0.9999    0.0044    0.0105
        //-0.0070    0.3000   -0.1000    1.0000    0.0873    0.2094
        //-0.1396
//                std::cout << "propagate pose param" << std::endl;
        obs_->Xv << 0.0300 << -0.0100 << 0.1000 << 0.9999 << 0.0044 << 0.0105 << -0.0070
                 << 0.3000 << -0.1000 << 1.0000 << 0.0873 << 0.2094 << -0.1396;
        obs_->predictPWLSVec(0.1, 1);

//                std::cout << "fill in kinematic block for verification" << std::endl;
        //        print out the full-seg system matrix:
        F_Q << 0.9999  << -0.0044  << -0.0105  <<  0.0070 << arma::endr
            << 0.0044  <<  0.9999  << -0.0070  << -0.0105 << arma::endr
            << 0.0105  <<  0.0070  <<  0.9999  <<  0.0044 << arma::endr
            << -0.0070  <<  0.0105 <<  -0.0044  <<  0.9999 << arma::endr;

        F_Omg << -0.0004 <<  -0.0010  <<  0.0007 << arma::endr
              << 0.0500 <<   0.0003 <<   0.0005 << arma::endr
              << -0.0004 <<   0.0500 <<  -0.0002 << arma::endr
              << -0.0005  <<  0.0002 <<   0.0500 << arma::endr;

        //        print out the in-seg system matrix:
        F_Q_inSeg << 1.0000  << -0.0003  << -0.0008  <<  0.0005 << arma::endr
                  << 0.0003  <<  1.0000 <<  -0.0005  << -0.0008 << arma::endr
                  << 0.0008  <<  0.0005  <<  1.0000  <<  0.0003 << arma::endr
                  << -0.0005  <<  0.0008 <<  -0.0003 <<   1.0000 << arma::endr;

        F_Omg_inSeg << -0.0000  << -0.0000 <<   0.0000 << arma::endr
                    << 0.0038   << 0.0000  <<  0.0000 << arma::endr
                    << -0.0000  <<  0.0038 <<  -0.0000 << arma::endr
                    << -0.0000 <<   0.0000  <<  0.0038 << arma::endr;
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    Observability * obs_;
    arma::mat F_Q, F_Omg, F_Q_inSeg, F_Omg_inSeg;
};

// When you have a test fixture, you define a test using TEST_F
// instead of TEST.

//// Tests the Measurement Jacobian.
//TEST_F(TestObsJacobian, Measurement) {

//}

// Tests the Kinematic/System Jacobian.
TEST_F(TestInstJacobian, InstKinematic) {
    //
    //    std::cout << "segment number " << obs_->kinematic.size() << std::endl;
    for (size_t i=0; i<1; ++i) {

//                std::cout << obs_->kinematic[i].F_Q  << std::endl;
//                std::cout << obs_->kinematic[i].F_Omg  << std::endl;
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Q - F_Q, "inf"), 0, 0.002);
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Omg - F_Omg, "inf"), 0, 0.0002);
        //
//                std::cout << obs_->kinematic[i].F_Q_inSeg  << std::endl;
//                std::cout << obs_->kinematic[i].F_Omg_inSeg  << std::endl;
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Q_inSeg - F_Q_inSeg, "inf"), 0, 0.0002);
        EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Omg_inSeg - F_Omg_inSeg, "inf"), 0, 0.0002);
    }
}

}  // namespace
