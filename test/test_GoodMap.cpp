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
class TestMapBounding : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // initialize the obs class
        //        std::cout << "start setting up" << std::endl;

        // EuRoC cam param
        double C2_f = 5.1369248;
        double C2_k1= 0; // -0.28340811;
        double C2_k2= 0; // 0.07395907;
        double C2_cx= 367.215;
        double C2_cy= 248.375;
        double C2_dx= 0.01123325985; // 0.0112;
        double C2_dy= 0.01123325985;
        double C2_nCols= 752;
        double C2_nRows= 480;

//        // Matlab cam param
//        double C2_f = 1.0;
//        double C2_k1= 0;
//        double C2_k2= 0;
//        double C2_cx= 320;
//        double C2_cy= 240;
//        double C2_dx= 1.0/320.0;
//        double C2_dy= 1.0/320.0;
//        double C2_nRows= 480;
//        double C2_nCols= 640;

        //        std::cout << "instantiate observability object" << std::endl;
        obs_ = new Observability(C2_f, C2_nRows, C2_nCols, C2_cx, C2_cy,
                                 C2_k1, C2_k2, C2_dx, C2_dy);

        //        std::cout << "propagate pose param" << std::endl;
        obs_->Xv << 0.3000  << -0.1000 <<   1.0000  <<  0.9992  <<  0.0131  <<  0.0314 <<  -0.0209
                 <<  3.0000 <<  -1.0000 <<  10.0000  <<  0.2618  <<  0.6283 <<  -0.4189;
        obs_->predictPWLSVec(0.1, 2);

        ORB_SLAM2::Frame::mnMinX = 0;
        ORB_SLAM2::Frame::mnMaxX = C2_nCols;
        ORB_SLAM2::Frame::mnMinY = 0;
        ORB_SLAM2::Frame::mnMaxY = C2_nRows;

        //        std::cout << "fill in map and measurement variables" << std::endl;
        // fill in necessary members
        size_t mapSize = 3000;
        //
        size_t i = 0;
        while (i < mapSize) {
            //
            cv::Mat lmkMat(3, 1, CV_32F);
            lmkMat.at<float>(0,0) = ( float(std::rand()) / float(RAND_MAX) ) * 12 - 6;
            lmkMat.at<float>(1,0) = ( float(std::rand()) / float(RAND_MAX) ) * 12 - 6;
            lmkMat.at<float>(2,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
//            lmkMat.at<float>(0,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
//            lmkMat.at<float>(1,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
//            lmkMat.at<float>(2,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8;

            if ( obs_->visible_Point_To_Frame(lmkMat, obs_->kinematic[1].Tcw) == true ) {
                // if visible, take the random lmk
                MapPoint * tmpMapPoint = new MapPoint;
                tmpMapPoint->SetWorldPos(lmkMat);
                poolMapPoints.push_back(tmpMapPoint);

                ++i;

            }
        }
    }

    // Declares the variables your tests want to use.
    Observability * obs_;
//    Frame * pFrame_;
    vector<MapPoint *> poolMapPoints;
};


TEST_F(TestMapBounding, MapBounding) {
    //
    std::cout << "subset selection from " << poolMapPoints.size() << " map points:" << std::endl;
    std::srand(std::time(nullptr));

    vector<double> timeArr, diffArr, sizeArr, maxArr, minArr;
    for (size_t num_good_inlier = 400; num_good_inlier <= 2400; num_good_inlier += 400) {
        //
//        std::cout << "======================= " << num_good_inlier << " / "
//                  << poolMapPoints.size() << " =======================" << std::endl;
        vector<GoodPoint> mpVec;
        vector<size_t> base_subset, curr_subset;
        size_t greedy_mtd;

        double avgTimeCost = 0, avgSubsetDiff = 0, maxSubsetDiff = -1,  minSubsetDiff = 99999;
        size_t repeat_no = 10; // 100; //
        double time_for_select = 1.0; // 0.06; // 0.015;
        for (size_t iter = 0; iter <= repeat_no; iter ++) {

            //            std::cout << "selection start!" << std::endl;
            obs_->mKineIdx = 1; // 0;
            obs_->mnFrameId = iter + 1;

            mpVec.clear();
            obs_->lmkSelectPool.clear();
            if (iter == 0) {
                greedy_mtd = 1;
                obs_->setSelction_Number(num_good_inlier, greedy_mtd, time_for_select, &poolMapPoints, &mpVec);
            }
            else {
                greedy_mtd = 3; // 2; // 4; //
                arma::wall_clock timer;
                timer.tic();

                bool flagSucc = obs_->setSelction_Number(num_good_inlier, greedy_mtd, time_for_select, &poolMapPoints, &mpVec);
                if (flagSucc == false)
                    std::cout << "selection failure! most likely due to time constraint!" << std::endl;

//                vector<size_t> indexVec;
//                for (size_t i=0; i<poolMapPoints.size(); ++i) {
//                    indexVec.push_back(i);
//                }
//                std::random_shuffle ( indexVec.begin(), indexVec.end() );
//                size_t pivotPos  = static_cast<size_t>(num_good_inlier);
//                if( indexVec.size() > pivotPos) {
//                    indexVec.erase(indexVec.begin() + pivotPos, indexVec.end());
//                }
//                for (size_t i=0; i<indexVec.size(); ++i) {
//                    GoodPoint tmpLmk(static_cast<size_t>(indexVec[i]), 1.0);
//                    mpVec.push_back(tmpLmk);
//                }

                //            std::cout << "selection done!" << std::endl;
                avgTimeCost += timer.toc();
            }

            curr_subset.clear();
            for (size_t i=0; i<mpVec.size(); ++i) {
                curr_subset.push_back(mpVec[i].idx);
            }
            std::sort(curr_subset.begin(), curr_subset.end());
//            std::cout << "-------------------------- Round " << iter
//                      << " --------------------------" << std::endl;

            if (iter == 0) {
                // create the base set
                base_subset = curr_subset;
            }
            else {
                // compute the set diff between repeats
                std::vector<size_t> diff_set;
                std::set_difference(base_subset.begin(), base_subset.end(), curr_subset.begin(), curr_subset.end(),
                                    std::inserter(diff_set, diff_set.begin()));
                avgSubsetDiff += diff_set.size();

                if (diff_set.size() > maxSubsetDiff)
                    maxSubsetDiff = diff_set.size();
                if (diff_set.size() < minSubsetDiff)
                    minSubsetDiff = diff_set.size();

//                std::cout << "Different lmk selected at current repeat: " << diff_set.size() << std::endl;
                EXPECT_NEAR(diff_set.size(), 0, ceil( base_subset.size() * 0.2 ));
            }
        }

        avgTimeCost = avgTimeCost / double(repeat_no);
        avgSubsetDiff = avgSubsetDiff / double(repeat_no);
//        std::cout << "Average time cost of subset selection = " << avgTimeCost << std::endl;
        timeArr.push_back(avgTimeCost);
        diffArr.push_back(avgSubsetDiff);
        sizeArr.push_back(num_good_inlier);
        maxArr.push_back(maxSubsetDiff);
        minArr.push_back(minSubsetDiff);
    }

    std::cout << std::endl << "Summay: " << std::endl;
    std::cout << "Subset / Full: " << std::endl;
    for (size_t i=0; i<timeArr.size(); ++i)
        std::cout << sizeArr[i] << "/" << poolMapPoints.size() << " ";
    std::cout << std::endl;

    std::cout << "Time Cost: " << std::endl;
    for (size_t i=0; i<timeArr.size(); ++i)
        std::cout << timeArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Average Subset Diff: " << std::endl;
    for (size_t i=0; i<diffArr.size(); ++i)
        std::cout << diffArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Max. Subset Diff: " << std::endl;
    for (size_t i=0; i<maxArr.size(); ++i)
        std::cout << maxArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Min. Subset Diff: " << std::endl;
    for (size_t i=0; i<minArr.size(); ++i)
        std::cout << minArr[i] << " ";
    std::cout << std::endl;
}

}  // namespace
