/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Hashing.h"
#include "Util.hpp"

#include <mutex>


//#define LOGGING_KF_LIST

//#define LOCAL_BA_TIME_LOGGING


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

struct BudgetPredictParam {
    double budget_per_frame;
    double coe_a, coe_b, coe_c, coe_d;
    //
    std::vector<double> match_ratio_log;
    size_t              match_ratio_idx;
    //
    double              avg_match_num;
    //    double              visible_mpt_num;
};

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    ~LocalMapping() {
        f_realTimeBA.close();
    }

    void SetLoopCloser(LoopClosing* pLoopCloser);
    void SetHashHandler(HASHING::MultiIndexHashing* pHashHandler);
    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    // void SetNeedUpdateHashTables(bool flag);

    bool NeedUpdateHashTables();

    void UpdateHashTables(std::vector<MapPoint*>& vpMPs);

#ifdef LOGGING_KF_LIST
    void SetRealTimeFileStream(string fNameRealTimeBA);
#endif

    // Time log
    std::vector<MappingLog> mvBATimeLog;
    MappingLog logCurrentKeyFrame;

#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    // Historical data for online budget estimation
    //    std::queue<std::pair<double, double>> mqBudgetHistory;
    //    Eigen::Matrix<double, NUM_HISTORICAL_BUDGET, 4> mKFNumMat;
    //    Eigen::Matrix<double, NUM_HISTORICAL_BUDGET, 1> mBudgetMat;
    size_t mRunningRow;
    bool   mbFullyLoaded;
#endif
    //
    BudgetPredictParam mParam;

protected:
    //    void UpdateHashTables(std::vector<MapPoint*>& vpMPs);
    //    void UpdateHashTables(std::list<MapPoint*>& vpMPs);

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

#ifdef LOGGING_KF_LIST
    vector<size_t> mvKeyFrameList;
    vector<size_t> mvFixedFrameList;
#endif

    arma::wall_clock timer;

    std::ofstream f_realTimeBA;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;
    HASHING::MultiIndexHashing* mpHashMethod;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;
    std::vector<MapPoint*> mvpNewAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
    std::mutex mMutexUpdateHashTables;
};

} // namespace ORB_SLAM2

#endif // LOCALMAPPING_H
