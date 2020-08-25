/**
* This file is part of GF-ORB-SLAM2.
*
* Copyright (C) 2019 Yipu Zhao <yipu dot zhao at gatech dot edu> 
* (Georgia Institute of Technology)
* For more information see 
* <https://sites.google.com/site/zhaoyipu/good-feature-visual-slam>
*
* GF-ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GF-ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GF-ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OBSERVABILITY_H
#define OBSERVABILITY_H

#include <iostream>
#include <assert.h>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <algorithm>
#include <thread>
#include <string>
#include <cstdlib>
#include <mutex>

#include "ORBmatcher.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Util.hpp"

#define EPS                             1e-6

#define DIMENSION_OF_STATE_MODEL        7 // 13 //

//#define OBS_SEGMENT_NUM                 1 // 7 // 3 //

//#define PROJ_DIST_SYSTEM_STATE_CHECK    50  // 30

//#define OBS_TIMECOST_VERBOSE
//#define OBS_DEBUG_VERBOSE

//#define OBS_MIN_SVD
//#define OBS_MAXVOL_TOP_SV
//#define OBS_MAXVOL_FULL
#define OBS_MAXVOL_Greedy

// Instead of scoring feature with information gain,
// use the information efficiency score
//#define INFORMATION_EFFICIENCY_SCORE

// Several extensions of measurement Jacobian
#define WITH_OCT_LEVELED_NOISE
//#define WITH_PROJECTION_JACOBIAN
//#define BASE_WEIGHT_QUAL                    0.5
//#define WITH_QUALITY_WEIGHTED_JACOBIAN
// seems to help at some cases; need to test on all 11 sequences before finalizing the robust term
// #define WITH_ROBUST_WEIGHTED_JACOBIAN

// The overhead of introducing the lock is on micro-seconds, which is neglectable
#define MULTI_THREAD_LOCK_ON

//#define LOCAL_JACOBIAN

#define Greedy_Paral_TrigSz             1000 // 1500 // 100 // 80 // 500 //
#define Greedy_Paral_RelFac             1.2 // 1.4 // 1.1 // 1.6 //

//#define MAX_TIMECOST_SELECT             0.003 // 0.002 // 0.0015 //

//#define RANDOM_SHUFFLE_LAZIER_GREEDY
#define RANDOM_ACCESS_LAZIER_GREEDY
#define MAX_RANDOM_QUERY_TIME          20 // 2000 // 100 //

// arma::log_det is 50% slower than the LU decomp version
// keep this marco commented plz
//#define ARMA_LOGDET

//#define PINV_STANDARD
//#define PINV_FAST
#define PINV_FAST_PERP

namespace ORB_SLAM2 {

//class System;

enum {
    MAP_INFO_MATRIX = 0,
    FRAME_INFO_MATRIX = 1,
    //
    MAP_HYBRID_MATRIX = 2,
    FRAME_HYBRID_MATRIX = 3,
    //
    FRAME_STRIP_OBS_MATRIX = 4
};

enum {
    BASELINE_RANDOM = 100,
    BASELINE_LONGLIVE = 101
};

// typedef std::vector<std::pair<arma::mat, arma::mat> >  Triplet;

class Observability {

public:

    Observability(double _f, int _nRows, int _nCols, double _Cx, double _Cy,
                  double _k1, double _k2, double _dx, double _dy, int _sensor):
        camera(_f,  _nRows,  _nCols,  _Cx,  _Cy, _k1,  _k2,  _dx,  _dy)
    {
        mSensor = _sensor;

        arma::colvec v1;
        v1 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
        dqbar_by_dq = arma::diagmat(v1);

        mBoundXInFrame = 20;
        mBoundYInFrame = 20;
        mBoundDepth = 0;

        mnFrameId = 0;

        mbNeedVizCheck = false;

        //        Twr = arma::zeros<arma::mat>(3, 1);
        //        Qwr = arma::zeros<arma::mat>(4, 1);     Qwr[0] = 1;
        //        tVel = arma::zeros<arma::mat>(3, 1);
        //        rVel = arma::zeros<arma::mat>(3, 1);

        mNumThreads = std::thread::hardware_concurrency(); // 4; // 
        if (mNumThreads > 1)
            mThreads = new std::thread [mNumThreads-1];
        maxEntryIdx = new size_t[mNumThreads];
        maxEntryVal = new double[mNumThreads];

        // NOTE
        // obselete in the future impl of insteneous obs analysis -----------------------
        //
        //        measPerSeg = 1; // 7; // 13;

        // H13 multiplicative factor
        H13_mul_fac = arma::ones<arma::mat>(2*13, 3);
        for(int fillIdx = 0; fillIdx < 13; fillIdx++) {
            H13_mul_fac.rows((fillIdx)*2, fillIdx*2 + 1) *= static_cast<double>(fillIdx);
        }

        mKineIdx = 0;
        Xv = arma::zeros<arma::rowvec>(13);

        // ------------------------------------------------------------------------------

        time_MatBuild = 0;
        time_Selection = 0;
    }


    Observability(double _fu, double _fv, int _nRows, int _nCols, double _Cx, double _Cy,
                  double _k1, double _k2, int _sensor):
        camera(_fu,  _fv, _nRows,  _nCols,  _Cx,  _Cy, _k1,  _k2)
    {
        mSensor = _sensor;

        arma::colvec v1;
        v1 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
        dqbar_by_dq = arma::diagmat(v1);

        mBoundXInFrame = 20;
        mBoundYInFrame = 20;
        mBoundDepth = 0;

        mnFrameId = 0;

        mbNeedVizCheck = false;

        //        Twr = arma::zeros<arma::mat>(3, 1);
        //        Qwr = arma::zeros<arma::mat>(4, 1);     Qwr[0] = 1;
        //        tVel = arma::zeros<arma::mat>(3, 1);
        //        rVel = arma::zeros<arma::mat>(3, 1);

        mNumThreads = std::thread::hardware_concurrency(); // 4; // 
        if (mNumThreads > 1)
            mThreads = new std::thread [mNumThreads-1];
        maxEntryIdx = new size_t[mNumThreads];
        maxEntryVal = new double[mNumThreads];

        // NOTE
        // obselete in the future impl of insteneous obs analysis -----------------------
        //
        //        measPerSeg = 1; // 7; // 13;

        // H13 multiplicative factor
        H13_mul_fac = arma::ones<arma::mat>(2*13, 3);
        for(int fillIdx = 0; fillIdx < 13; fillIdx++) {
            H13_mul_fac.rows((fillIdx)*2, fillIdx*2 + 1) *= static_cast<double>(fillIdx);
        }

        mKineIdx = 0;
        Xv = arma::zeros<arma::rowvec>(13);

        // ------------------------------------------------------------------------------

        time_MatBuild = 0;
        time_Selection = 0;
    }

    ~Observability() {
	   if (mNumThreads > 1)
           delete [] mThreads;
    }

    // ============================ Kinematic func =============================

    // FIXME: may be let Xv be an input reference?
    //
    // extend the update func to future multi segments
    void updatePWLSVec (const double & time_prev, const cv::Mat& Tcw_prev,
                        const double & time_cur, const cv::Mat& Twc_cur) {

        //    arma::mat curXv;
        convert_Homo_Pair_To_PWLS_Vec(time_prev, Tcw_prev, time_cur, Twc_cur, this->Xv);

        // fill in the homogenenous form
        this->Twc = arma::zeros<arma::mat>(4, 4);
        for (int i=0; i<4; ++i) {
            for (int j=0; j<4; ++j) {
                //
                this->Twc(i, j) = Twc_cur.at<float>(i, j);
            }
        }

        this->Tcw_prev = arma::zeros<arma::mat>(4, 4);
        for (int i=0; i<4; ++i) {
            for (int j=0; j<4; ++j) {
                //
                this->Tcw_prev(i, j) = Tcw_prev.at<float>(i, j);
            }
        }

#if defined LOCAL_JACOBIAN
        // convert Xv into local version
        // Xv_rel
        this->Xv_rel = this->Xv;
        // r
        this->Xv_rel(0,0) = T_rel.at<float>(0, 3);
        this->Xv_rel(1,0) = T_rel.at<float>(1, 3);
        this->Xv_rel(2,0) = T_rel.at<float>(2, 3);
        // q
        this->Xv_rel.rows(3, 6) = DCM2QUAT_float( T_rel.rowRange(0,3).colRange(0,3) );
        //    std::cout << "Xv_rel = " << Xv_rel << std::endl;

#endif

    }

    void predictPWLSVec(const double & dt, const size_t & num_seg_pred) {

        arma::rowvec curXv = this->Xv;

        // iteratively propagating the PWLS state and estimating system matrix accordingy
        kinematic.clear();
        for (size_t i=0; i<num_seg_pred; ++i) {

            KineStruct tmpKine;
            tmpKine.dt = float(dt);
            tmpKine.dt_inSeg = tmpKine.dt / float(13);
            tmpKine.Xv = curXv;

            convert_PWLS_Vec_To_Homo(tmpKine.Xv, tmpKine.Tcw);
            // compute F block for full segment kinematic
            compute_F_subblock(tmpKine.Xv, tmpKine.dt_inSeg, tmpKine.F_Q_inSeg, tmpKine.F_Omg_inSeg);
            // assemble into full F matrix for current segment
            assemble_F_matrix(tmpKine.dt_inSeg, tmpKine.F_Q_inSeg, tmpKine.F_Omg_inSeg, tmpKine.F_inSeg);
            // compute F block for each interpolation point within the segment
            compute_F_subblock(tmpKine.Xv, tmpKine.dt, tmpKine.F_Q, tmpKine.F_Omg);
            assemble_F_matrix(tmpKine.dt, tmpKine.F_Q, tmpKine.F_Omg, tmpKine.F);

            // update Xv
            propagate_PWLS(tmpKine, curXv);
            //        curXv = tmpKine.F * curXv.t();

            // save the current state
            kinematic.push_back(tmpKine);
        }
    }

    // ============================ Measurement func =============================

    inline void project_Point_To_Frame(const cv::Mat & Pw, const cv::Mat & mTcw,
                                       float & u, float & v, arma::mat & ProjJacob) {

        // 3D in camera coordinates
        cv::Mat mRcw_tmp = mTcw.rowRange(0,3).colRange(0,3);
        cv::Mat mtcw_tmp = mTcw.rowRange(0,3).col(3);

        const cv::Mat Pc = mRcw_tmp * Pw + mtcw_tmp; // mRcw_tmp.t() * ( Pw - mtcw_tmp ); //
        const float PcX = Pc.at<float>(0);
        const float PcY = Pc.at<float>(1);
        const float PcZ = Pc.at<float>(2);

        //    std::cout << PcX << ", " << PcY << ", " <<  PcZ << std::endl;

        // Check positive depth
        if(PcZ < 0.0) {
            u = -1;
            v = -1;
            return ;
        }

        //    std::cout << "fu = " << fu << " fv = " << fv << " Cx = " << Cx << " Cy = " << Cy << std::endl;

        // Project in image and check it is not outside
        u = float(camera.fu) * PcX/PcZ + float(camera.Cx);
        v = float(camera.fv) * PcY/PcZ + float(camera.Cy);

        //    std::cout << u << ", " << v << std::endl;

        if (u<0 || u>camera.nCols || v<0 || v>camera.nRows){
            u = -1;
            v = -1;
            return ;
        }


        double PcZ_2  = PcZ * PcZ;
        //
        ProjJacob << camera.fu/PcZ << 0.0f << - camera.fu * PcX / PcZ_2 << arma::endr
                  << 0.0f << camera.fv/PcZ << - camera.fv * PcY / PcZ_2 << arma::endr
                  << 0.0f << 0.0f << 1.0f << arma::endr;

        //    std::cout << "proj pixel = [" << pt2D->pt.x << ", " << pt2D->pt.y << "]; meas. pixel = [" << u << ", " << v << "]" << std::endl;

        return ;
    }

    inline bool visible_Point_To_Frame(const cv::Mat & Pw, const cv::Mat & mTcw) {

        // 3D in camera coordinates
        cv::Mat mRcw_tmp = mTcw.rowRange(0,3).colRange(0,3);
        cv::Mat mtcw_tmp = mTcw.rowRange(0,3).col(3);

        const cv::Mat Pc = mRcw_tmp * Pw + mtcw_tmp; // mRcw_tmp.t() * ( Pw - mtcw_tmp ); //
        const float PcX = Pc.at<float>(0);
        const float PcY = Pc.at<float>(1);
        const float PcZ = Pc.at<float>(2);
        //    std::cout << PcX << ", " << PcY << ", " <<  PcZ << std::endl;

        // Check positive depth
        if(PcZ < 0.0) {
            return false;
        }

        // Project in image and check it is not outside
        float u = float(camera.fu) * PcX/PcZ + float(camera.Cx),
                v = float(camera.fv) * PcY/PcZ + float(camera.Cy);

        if (u<0 || u>camera.nCols || v<0 || v>camera.nRows){
            return false;
        }

        return true;
    }


    // No visibility check is done in this func, with the assumption that input Xv is already visible
    inline void compute_H_subblock_complete (const arma::rowvec & Xv,
                                             const arma::rowvec & yi,
                                             const arma::rowvec & zi,
                                             arma::mat & H13, arma::mat & H47,
                                             arma::mat & dhu_dhrl,
                                             float & residual_u, float & residual_v) {
        arma::rowvec q_wr = Xv.subvec(3, 6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0,2);
//        std::cout << "yi = " << yi << "; Xv = " << Xv << "; t_rw = " << t_rw  << "; R_rw = " << R_rw << std::endl;

        //    arma::mat dhd_dhu(2, 2, arma::fill::eye);
        //    cout << dhd_dhu << endl;

        // dhd_dhu
        double ud = zi[0];
        double vd = zi[1];
        double xd = (zi[0] - camera.Cx) * camera.dx;
        double yd = (zi[1] - camera.Cy) * camera.dy;
        double rd2 = (xd * xd) + (yd * yd);
        double rd4 = rd2 * rd2;
        double uu_ud = (1+camera.k1*rd2+camera.k2*rd4)+(ud-camera.Cx)*(camera.k1+2*camera.k2*rd2)*(2*(ud-camera.Cx)*camera.dx*camera.dx);
        double vu_vd = (1+camera.k1*rd2+camera.k2*rd4)+(vd-camera.Cy)*(camera.k1+2*camera.k2*rd2)*(2*(vd-camera.Cy)*camera.dy*camera.dy);
        double uu_vd = (ud-camera.Cx)*(camera.k1+2*camera.k2*rd2)*(2*(vd-camera.Cy)*camera.dy*camera.dy);
        double vu_ud = (vd-camera.Cy)*(camera.k1+2*camera.k2*rd2)*(2*(ud-camera.Cx)*camera.dx*camera.dx);

        arma::mat J_undistor;
        //    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;
        J_undistor = { {uu_ud, uu_vd}, {vu_ud, vu_vd} };
        arma::mat dhd_dhu = J_undistor.i();

        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();
//        std::cout << "hrl = " << hrl << endl;

        //    arma::mat dhu_dhrl;
        if ( fabs(hrl(2,0)) < 1e-6 ) {
            dhu_dhrl  = arma::zeros<arma::mat>(2,3);
        } else {
            //        dhu_dhrl << fu/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*fu/( std::pow(hrl(2,0), 2.0)) << arma::endr
            //                 << 0.0    <<  fv/(hrl(2,0))   <<  -hrl(1,0)*fv/( std::pow(hrl(2,0), 2.0)) << arma::endr;
            dhu_dhrl = { {camera.fu/(hrl(2,0)), 0.0, -hrl(0,0)*camera.fu/( std::pow(hrl(2,0), 2.0))},
                         {0.0, camera.fv/(hrl(2,0)), -hrl(1,0)*camera.fv/( std::pow(hrl(2,0), 2.0))} };
        }
//        cout << "dhu_dhrl = " << dhu_dhrl << endl;

        // as a sanity check, we can compare hrl with zi
        const float invz = 1.0/hrl(2,0);
        const float u = float(camera.fu)*hrl(0,0)*invz + float(camera.Cx);
        const float v = float(camera.fv)*hrl(1,0)*invz + float(camera.Cy);
        residual_u = fabs(zi[0] - u);
        residual_v = fabs(zi[1] - v);
        //    std::cout << "sanity check of H matrix: " << u << ", " << v << "; " << zi << std::endl;
        //    std::cout << "sanity check of H matrix: " << (fabs(zi(0,0) - u) + fabs(zi(0,1) - v)) << std::endl;
        //    if ((fabs(zi(0,0) - u) + fabs(zi(1,0) - v)) > PROJ_DIST_SYSTEM_STATE_CHECK) {
        //        // current system state being off too much
        //        // discard the obs info
        //        H13.zeros();
        //        H47.zeros();
        //        return ;
        //    }

        arma::mat dh_dhrl = dhd_dhu * dhu_dhrl;
        arma::rowvec qwr_conj = qconj(q_wr);
        //        cout << "qwr_conj = " << qwr_conj << endl;

        // H matrix subblock (cols 1~3): H13
        H13 = -1.0 * (dh_dhrl *  R_rw);
//        cout << "H13 = " << H13 << endl;

//        cout << "dh_dhrl = " << dh_dhrl << endl;
//        cout << "dRq_times_a_by_dq = " << dRq_times_a_by_dq( qwr_conj ,  t_rw) << endl;
//        cout << "dqbar_by_dq = " << dqbar_by_dq << endl;

        // H matrix subblock (cols 4~7): H47
        H47 = dh_dhrl * (dRq_times_a_by_dq( qwr_conj ,  t_rw) * dqbar_by_dq);
//        cout << "H47 = " << H47 << endl;

        return;
    }

    // Visibility check is needed here, given that the input Xv could be all local map points before data association
    // To cope for the possible prediction error, we increase the visibility range here
    inline bool compute_H_subblock_simplied (const arma::rowvec & Xv,
                                             const arma::rowvec & yi,
                                             arma::mat & H13, arma::mat & H47,
                                             arma::mat & dhu_dhrl,
                                             const bool check_viz,
                                             float & u, float & v) {

        arma::rowvec q_wr = Xv.subvec(3,6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0,2);
        //    std::cout << "yi = " << yi << "; Xv = " << Xv << "; RelRw = " << RelRw << std::endl;

        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();

        if (hrl(2,0) > 0) {
            u = float(camera.fu) * hrl(0,0)/hrl(2,0) + float(camera.Cx);
            v = float(camera.fv) * hrl(1,0)/hrl(2,0) + float(camera.Cy);
        }
        else {
            u = FLT_MAX;
            v = FLT_MAX;
        }

        // check visibility
        if (check_viz) {
            //
            if ( hrl(2,0) < 0.0 + this->mBoundDepth )
                return false;
            if(u < ORB_SLAM2::Frame::mnMinX - this->mBoundXInFrame || u > ORB_SLAM2::Frame::mnMaxX + this->mBoundXInFrame)
                return false;
            if(v < ORB_SLAM2::Frame::mnMinY - this->mBoundYInFrame || v > ORB_SLAM2::Frame::mnMaxY + this->mBoundYInFrame)
                return false;
        }

        //    arma::mat dhu_dhrl;
        if ( fabs(hrl(2,0)) < 1e-6 ) {
            dhu_dhrl  = arma::zeros<arma::mat>(2,3);
        } else {
            //        dhu_dhrl << fu/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*fu/( std::pow(hrl(2,0), 2.0)) << arma::endr
            //                 << 0.0    <<  fv/(hrl(2,0))   <<  -hrl(1,0)*fv/( std::pow(hrl(2,0), 2.0)) << arma::endr;
            dhu_dhrl = { {camera.fu/(hrl(2,0)), 0.0, -hrl(0,0)*camera.fu/( std::pow(hrl(2,0), 2.0))},
                         {0.0, camera.fv/(hrl(2,0)), -hrl(1,0)*camera.fv/( std::pow(hrl(2,0), 2.0))} };
        }

        arma::rowvec qwr_conj = qconj(q_wr);

        // H matrix subblock (cols 1~3): H13
        H13 = -1.0 * (dhu_dhrl *  R_rw);

        // H matrix subblock (cols 4~7): H47
        H47 = dhu_dhrl * (dRq_times_a_by_dq( qwr_conj ,  t_rw) * dqbar_by_dq);

        return true;
    }

    inline void reWeightInfoMat(const Frame * F, const int & kptIdx, const MapPoint * pMP,
                                const arma::mat & H_meas, const float & res_u, const float & res_v,
                                const arma::mat & H_proj, arma::mat & H_rw) {

        int measSz = H_meas.n_rows;
        arma::mat Sigma_r(measSz, measSz), W_r(measSz, measSz);
        Sigma_r.eye();

        if (F != NULL && kptIdx >= 0 && kptIdx < F->mvKeysUn.size()) {
#ifdef WITH_OCT_LEVELED_NOISE
            //
            float Sigma2 = F->mvLevelSigma2[F->mvKeysUn[kptIdx].octave];
            Sigma_r = Sigma_r * Sigma2;
#ifdef OBS_DEBUG_VERBOSE
            std::cout << Sigma_r << std::endl;
#endif

#endif
        }

#ifdef WITH_PROJECTION_JACOBIAN
        //
        double stdMapErr = -exp(double(mMapPoints->at(i)->mnVisible - 1)) * 0.01;
        Sigma_r = Sigma_r + H_proj * H_proj.t() * std::pow(stdMapErr, 2.0);
#ifdef OBS_DEBUG_VERBOSE
        std::cout << Sigma_r << std::endl;
#endif

#endif

        // cholesky decomp of diagonal-block scaling matrix W
        if (arma::chol(W_r, Sigma_r, "lower") == true) {
            // scale the meas. Jacobian with the scaling block W_r
            H_rw = arma::inv(W_r) * H_meas;
        }
        else {
            // do nothing
            //                    std::cout << "chol failed!" << std::endl;
            //                    std::cout << "oct level =" << kpUn.octave << "; invSigma2 = " << invSigma2 << std::endl;
        }
#ifdef OBS_DEBUG_VERBOSE
        std::cout << W_r << std::endl;
        std::cout << H_rw << std::endl << std::endl << std::endl;
#endif

//#ifdef WITH_QUALITY_WEIGHTED_JACOBIAN
//        double quality_max = double(ORBmatcher::TH_HIGH);
//#ifdef OBS_DEBUG_VERBOSE
//        std::cout << F->mvpMatchScore[i] << std::endl;
//        std::cout << H << std::endl;
//#endif
//        double weight_qual = std::max(0.0, double(quality_max - F->mvpMatchScore[i]) / double(quality_max));
//        weight_qual = weight_qual * (1.0 - BASE_WEIGHT_QUAL) + BASE_WEIGHT_QUAL;
//        H = H * weight_qual;
//#ifdef OBS_DEBUG_VERBOSE
//        std::cout << weight_qual << std::endl;
//        std::cout << H << std::endl << std::endl << std::endl;
//#endif
//#endif

#ifdef WITH_ROBUST_WEIGHTED_JACOBIAN
        //
        // NOTE
        // weight the measurement Jacobain with huber loss weight
        // use the same delta value in ORB-SLAM g2o optimization
        //
        float weight_u, weight_v;
        compute_Huber_weight( res_u, weight_u );
        compute_Huber_weight( res_v, weight_v );
        H_rw.row(0) = H_rw.row(0) * weight_u;
        H_rw.row(1) = H_rw.row(1) * weight_v;
        //                if ( res_u > sqrt(5.991) || res_v > sqrt(5.991) ) {
        //                    std::cout << "residu = " << res_u << ", " << res_v << std::endl;
        //                    std::cout << "weight = " << weight_u << ", " << weight_v << std::endl;
        //                }

#endif

    }

    inline void compute_H_disparity_col (const arma::rowvec & Xv,
                                         const arma::rowvec & yi,
                                         arma::mat & H_disp) {

        arma::rowvec q_wr = Xv.subvec(3,6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0,2);

        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();

        arma::mat ddisp_dhrl;
        if ( fabs(hrl(2,0)) < 1e-6 ) {
            ddisp_dhrl  = arma::zeros<arma::mat>(1,3);
        } else {
            ddisp_dhrl = { 0.0, 0.0, -camera.bf/( std::pow(hrl(2,0), 2.0)) };
        }

        arma::rowvec qwr_conj = qconj(q_wr);

        H_disp = arma::join_horiz( -1.0 * (ddisp_dhrl *  R_rw), ddisp_dhrl * (dRq_times_a_by_dq( qwr_conj ,  t_rw) * dqbar_by_dq) );

        return ;
    }


    // ============================ Matrix construction func =============================

    bool runMatrixBuilding(const size_t mat_type, const double time_for_build, const bool with_multi_thread, const bool check_viz = false);

    void batchStripObsMat_Frame(const int start_idx, const int end_idx);

    //
    void batchInfoMat_Frame(const size_t start_idx, const size_t end_idx, const double time_for_build);

    void batchInfoMat_Map(const size_t start_idx, const size_t end_idx, const double time_for_build, const bool check_viz);

    //
    void batchHybridMat_Frame(const size_t start_idx, const size_t end_idx, const double time_for_build);

    void batchHybridMat_Map(const size_t start_idx, const size_t end_idx, const double time_for_build, const bool check_viz);

    void compute_SOM_In_Segment(const size_t seg_idx, const arma::mat Y, const arma::mat Z,
                                arma::mat & curObsMat);

    // ============================ Active matching func =============================

    int runActiveMapMatching(Frame *pFrame,
                             const size_t mat_type,
                             const arma::mat &mBaseInfoMat,
                             const float th, ORBmatcher &mORBMatcher,
                             const int num_to_match, const double time_for_match );

    int runBaselineMapMatching(Frame *pFrame,
                               const size_t base_mtd,
                               const float th,
                               ORBmatcher &mORBMatcher,
                               const int num_to_match,
                               const double time_for_match );

    // private:

    // common/ anxilary variables
    arma::mat dqbar_by_dq;
    arma::mat H13_mul_fac; // ObsMat.cols(7,9) = repmat(H13, 13, 1)% H13_mul_fac * dt

    long unsigned int mnFrameId;

    std::vector<MapPoint*> * mMapPoints;
    std::vector<MapPoint*> mLeftMapPoints;
    bool mbNeedVizCheck;

    PinHoleCamera camera;

    int mSensor;

    int mBoundXInFrame, mBoundYInFrame;
    float mBoundDepth;

    //
    Frame * pFrame;
    arma::rowvec Xv, Xv_rel;
    arma::mat Twc, Tcw_prev;
    //    arma::mat Twr, Qwr, tVel, rVel;
    //    size_t measPerSeg;
    //    size_t segNum;
    //    int measPerSeg;
    //    int segNum;

    size_t mKineIdx;
    vector<KineStruct> kinematic;
    //    double dt, dt_inSeg;
    //    arma::mat F_Q, F_Omg, F;
    //    arma::mat F_Q_inSeg, F_Omg_inSeg, F_inSeg;


    vector<GoodPoint> lmkSelectPool;

    // multi-thread variables
    size_t mNumThreads;
    std::thread * mThreads;
    // for tmp result saving
    size_t * maxEntryIdx;
    double * maxEntryVal;
    // for random permutation
    std::vector<size_t> rndEntryIdx;
    // iteratively search for the most informative lmk
    arma::mat baseInfoMat;
    // create a query index for fast insert and reject of entries
    arma::mat queryLmkIdx;

    vector<double> maxPerpDistArr;
    vector<int> maxPerpIdxArr;

    // time log
    double time_MatBuild;
    double time_Selection;

};

}


//class LmkSelectionInfo {
//public:
//    //    ObsThresCell(mCurrentFrame.mTimeStamp, T_ref, Q_ref,
//    //                 best_obs_thre, NumPtsUsedLast, mPtObs.size(),
//    //                 T_opt, Q_opt, min_err_T, min_err_R, ini_err_T, ini_err_R)
//    LmkSelectionInfo(const double &time_stamp,
//                     const vector<float> &T_init, const vector<float> &Q_init, const int &num_lmk_init,
//                     const vector<float> &T_best, const vector<float> &Q_best, const int &num_lmk_best,
//                     const double &thre_obs_best, vector<GoodPoint> &lmk_arr) {
//        //
//        this->time_stamp = time_stamp;
//        for (int i=0; i<3; ++i)
//            this->T_init[i] = T_init[i];
//        for (int i=0; i<4; ++i)
//            this->Q_init[i] = Q_init[i];
//        this->num_lmk_init = num_lmk_init;
//        //
//        for (int i=0; i<3; ++i)
//            this->T_best[i] = T_best[i];
//        for (int i=0; i<4; ++i)
//            this->Q_best[i] = Q_best[i];
//        this->num_lmk_best = num_lmk_best;
//        //
//        this->thre_obs_best = thre_obs_best;
//        if (lmk_arr.size() > 0)
//            this->lmk_arr = new std::vector<GoodPoint>(lmk_arr);
//        else
//            this->lmk_arr = NULL;
//    }

//    double time_stamp;
//    double T_init[3];
//    double Q_init[4];
//    double T_best[3];
//    double Q_best[4];
//    //
//    int num_lmk_init;
//    int num_lmk_best;
//    //
//    double thre_obs_best;
//    vector<GoodPoint> * lmk_arr;
//    //    double err_T_best;
//    //    double err_R_best;
//    //    double err_T_init;
//    //    double err_R_init;
//    //
//};

#endif // OBSERVABILITY_H
