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

#include <stdio.h>
#include "System.h"
#include <opencv2/highgui/highgui.hpp>
#include "gtest/gtest.h"

using namespace ORB_SLAM2;

namespace {
// To use a test fixture, derive a class from testing::Test.
class TestMapIO : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        //
        string strVocFile = "../ORB_Data/ORBvoc.bin";
        //
        string strSettingPath = "../ORB_Data/EuRoC_yaml/EuRoC_lmk1000.yaml";
        string path_to_image_folder = "/mnt/DATA/Datasets/EuRoC_dataset/MH_02_easy/cam0/data/";
        string path_to_image_times = "/mnt/DATA/Datasets/EuRoC_dataset/MH_02_easy/cam0/times.txt";

        SLAM_ = new System(strVocFile, strSettingPath, System::MONOCULAR, true);
//        SLAM_ = new System(strVocFile, strSettingPath, System::MONOCULAR, false);

        // Retrieve paths to images
        LoadImages(path_to_image_folder, path_to_image_times);

        nImages = vstrImageFilenames.size(); // 500; //


    }

    void LoadImages(const std::string &strImagePath, const std::string &strPathTimes)
    {
        ifstream fTimes;
        fTimes.open(strPathTimes.c_str(), ios::in);
        vTimestamps.reserve(5000);
        vstrImageFilenames.reserve(5000);
        while(!fTimes.eof())
        {
            std::string s;
            getline(fTimes,s);
            if(!s.empty())
            {
//                cout << s << endl;
                char fname[200];
                long unsigned int t_;
                sscanf(s.c_str(), "%lu,%s", &t_, fname);
                std::string s_ = strImagePath + "/" + std::string(fname);
//                cout << s_.c_str() << endl;
                vstrImageFilenames.push_back(s_);
                //                cout << t_ << ", " << double(t_)/1e9 << endl;
                vTimestamps.push_back(double(t_)/1e9);

                //                stringstream ss;
                //                ss << s;
                //                vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
                //                cout << ss.str() << endl;
                //                //
                //                double t;
                //                ss >> t;
                //                vTimeStamps.push_back(t/1e9);
                //                cout << t/1e9 << endl;
            }
        }
    }

    void RunImages() {

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        // Main loop
        cv::Mat im;
        for(int ni=0; ni<nImages; ni++)
        {
            cout << vstrImageFilenames[ni] << endl;
//            cout << vstrImageFilenames[ni] << "; " << vTimestamps[ni] << endl;

            // Read image from file
            im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[ni] << endl;
                return ;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            // Pass the image to the SLAM system
            SLAM_->TrackMonocular(im,tframe);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    System * SLAM_;
    int nImages;

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
};


//TEST_F(TestMapIO, Save) {
//    //
//    RunImages();
//    SLAM_->SaveMap(std::string("/mnt/DATA/tmp/MapIO/MH_01"));
//    SLAM_->Shutdown();
//}

TEST_F(TestMapIO, Load) {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    SLAM_->LoadMap(std::string("/mnt/DATA/tmp/MapIO/MH_aggr/"));

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double tload = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    cout << endl << "Time to load the map = " << tload << " sec" << endl;

    SLAM_->ForceRelocTracker();

    RunImages();

//    SLAM_->SaveMap(std::string("/mnt/DATA/tmp/MapIO/MH_aggr_2"));

    SLAM_->Shutdown();
}

}  // namespace
