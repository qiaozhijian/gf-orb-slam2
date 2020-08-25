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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#define ARMA_NO_DEBUG
#include "armadillo"

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

// macros defined to enforce budget on feature matching per frame
//#define BUDGETING_FEATURE_MATCHING
//#define MAX_NUM_FEATURE_MATCHING        150

namespace ORB_SLAM2
{

class ORBmatcher
{
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);


    //    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    //    // Good features are searched with a coarser radius than the rest
    //    // Used to track the local map (Tracking)
    //    int SearchByProjection_GoodFeature(Frame &F, const vector<MapPoint*> &vpMapPoints,
    //                                       const float th_good, const float th_rest);

    //    int SearchByProjection_GoodFeature(Frame &CurrentFrame, const Frame &LastFrame,
    //                                       const float th_good, const float th_rest);


    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints, const float th = 3);

    //
    int SearchByProjection_Budget(Frame &F, const vector<MapPoint *> &vpMapPoints,
                                  const float th, const double time_constr);

    //
    inline int SearchByProjection_OnePoint(Frame &F, MapPoint *pMP, const float th)
    {
        const bool bFactor = th != 1.0;

        if (!pMP->mbTrackInView)
            return -1;

        if (pMP->isBad())
            return -1;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if (bFactor)
            r *= th;

        const vector<size_t> vIndices =
            F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1, nPredictedLevel);

        if (vIndices.empty())
            return -1;

        cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1;

        // Get best and second matches with near keypoints
        for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
        {
            const size_t idx = *vit;

            if (F.mvpMapPoints[idx])
                if (F.mvpMapPoints[idx]->Observations() > 0)
                    continue;

            if (F.mvuRight[idx] > 0)
            {
                const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
                if (er > r * F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor, d);

            if (dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx = idx;
            }
            else if (dist < bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2 = dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if (bestDist <= TH_HIGH)
        {
            if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                return -1;

            F.mvpMapPoints[bestIdx] = pMP;
            // store the match score for each frame-to-map match
            F.mvpMatchScore[bestIdx] = bestDist;

            return bestIdx;
        }
        else
            return -1;
    }

    inline void GetCandidates(Frame &F, MapPoint *pMP, const float th)
    {
        const bool bFactor = th != 1.0;

        if (!pMP->mbTrackInView)
            return;

        if (pMP->isBad())
            return;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if (bFactor)
            r *= th;

        pMP->mvMatchCandidates =
            F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1, nPredictedLevel);
    }

    inline int MatchCandidates(Frame &F, MapPoint *pMP, const float th)
    {

        if (!pMP->mbTrackInView)
            return -1;

        if (pMP->isBad())
            return -1;

        if (pMP->mvMatchCandidates.empty())
            return -1;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        const bool bFactor = th != 1.0;
        if (bFactor)
            r *= th;

        cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1;

        // Get best and second matches with near keypoints
        for (vector<size_t>::iterator vit = pMP->mvMatchCandidates.begin(), vend = pMP->mvMatchCandidates.end(); vit != vend; vit++)
        {
            const size_t idx = *vit;

            if (F.mvpMapPoints[idx])
                if (F.mvpMapPoints[idx]->Observations() > 0)
                    continue;

            if (F.mvuRight[idx] > 0)
            {
                const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
                if (er > r * F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            cv::Mat d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor, d);

            if (dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx = idx;
            }
            else if (dist < bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2 = dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if (bestDist <= TH_HIGH)
        {
            if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                return -1;

            F.mvpMapPoints[bestIdx] = pMP;

            // store the match score for each frame-to-map match
            F.mvpMatchScore[bestIdx] = bestDist;

            return bestIdx;
        }
        else
            return -1;
    }

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono, double & numVisibleMpt);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const std::set<MapPoint *> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    int SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, std::vector<MapPoint *> &vpMatched, int th);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches);
    int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th = 3.0);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

} // namespace ORB_SLAM2

#endif // ORBMATCHER_H
