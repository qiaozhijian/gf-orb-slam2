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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

#ifdef ENABLE_MAP_IO
    // For Map IO only; init keyframe from cv::FileStorage
    KeyFrame(cv::FileStorage &fs, Map *mMap, ORBVocabulary *mVocabulary, KeyFrameDatabase *mKeyFrameDatabase);
#endif

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    // For virtual KF creation only; cannot be used in regular KF processing (mapping; loop closing)
    KeyFrame(const double &timeStamp, const cv::Mat &Tcw,
             const float &fx_, const float &fy_, const float &cx_, const float &cy_, const float &mb_);
#endif

    void ExportToYML(cv::FileStorage &fs);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetOw();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame *pKF, const int &weight);
    void EraseConnection(KeyFrame *pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame *pKF);

    // Spanning tree functions
    void AddChild(KeyFrame *pKF);
    void EraseChild(KeyFrame *pKF);
    void ChangeParent(KeyFrame *pKF);
    std::set<KeyFrame *> GetChilds();
    KeyFrame *GetParent();
    bool hasChild(KeyFrame *pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame *pKF);
    std::set<KeyFrame *> GetLoopEdges();

    // MapPoint observation functions
    void AppendMapPoint(MapPoint *pMP);
    void AddMapPoint(MapPoint *pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint *pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
    std::set<MapPoint *> GetMapPoints();
    std::vector<MapPoint *> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint *GetMapPoint(const size_t &idx);
    size_t GetMatchNum();

    //
    void AddCovisibleKeyFrames(KeyFrame *pKF);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp(const int a, const int b)
    {
        return a > b;
    }

    static bool idComp(const KeyFrame *pKF1, const KeyFrame *pKF2)
    {
        return pKF1->mnId < pKF2->mnId;
    }

    static bool timeStampComp(const KeyFrame *pKF1, const KeyFrame *pKF2)
    {
        return pKF1->mTimeStamp < pKF2->mTimeStamp;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    
    // optimizable flag: free KF or fixed KF
    bool mbFixedKF;

#if defined ENABLE_MAP_IO || defined ENABLE_ANTICIPATION_IN_GRAPH
    long unsigned int mnFrameId;

    double mTimeStamp;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
    
#else
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;
#endif

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKFCand;
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKFCand;
    long unsigned int mnBAFixedForKF;
    //
    long unsigned int mnBALocalCount;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

#if defined ENABLE_MAP_IO || defined ENABLE_ANTICIPATION_IN_GRAPH
    // Calibration parameters
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth;  // negative value for monocular points
    cv::Mat mDescriptors;
#else
    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth;  // negative value for monocular points
    const cv::Mat mDescriptors;
#endif

    std::vector<cv::Mat> mvTrel;
    double mNumVisibleMpt;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

#if defined ENABLE_MAP_IO || defined ENABLE_ANTICIPATION_IN_GRAPH
    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;
#else
    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;
#endif

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint *> mvpMapPoints;

    // BoW
    KeyFrameDatabase *mpKeyFrameDB;
    ORBVocabulary *mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t>>> mGrid;

    std::map<KeyFrame *, int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame *mpParent;
    std::set<KeyFrame *> mspChildrens;
    std::set<KeyFrame *> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    Map *mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} // namespace ORB_SLAM2

#endif // KEYFRAME_H
