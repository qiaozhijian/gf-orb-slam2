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

#include "Observability.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <iosfwd>

//#define TIME_TMPRUPDATE_SORE
std::mutex mtx;

namespace ORB_SLAM2 {

void Observability::compute_SOM_In_Segment(const size_t seg_idx, const arma::mat Y, const arma::mat Z,
                                           arma::mat & curObsMat) {
    // Compute the H matrix
    arma::mat H13, H47, H_proj;
    float res_u = 0, res_v = 0;
    compute_H_subblock_complete(this->kinematic[seg_idx].Xv, Y, Z, H13, H47, H_proj, res_u, res_v);

    //    std::cout << "H13 = " << H13 << "; H47 = " << H47 << std::endl;

    //std::cout << "[OBS_COMPUTOR]  LinObsMat" << std::endl;
    curObsMat = arma::zeros<arma::mat>(26, 13);
    // Copy first 3 columns
    curObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
    // Columns 7~9
    curObsMat.cols(7, 9) = curObsMat.cols(0, 2) % this->H13_mul_fac;
    if(fabs(this->kinematic[seg_idx].dt_inSeg - 1.0) > 1e-6) {
        curObsMat.cols(7,9) = curObsMat.cols(7,9) * this->kinematic[seg_idx].dt_inSeg;
    }
    // First segment in linObsMat: just H:
    curObsMat(arma::span(0,1), arma::span(3,6))= H47;

    // Segment 2 to 13
    arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
    arma::mat rollingQFac = this->kinematic[seg_idx].F_Q_inSeg;  // Q^n
    for (int j = 1; j < 13; j++) {
        // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

        // 2nd block:  (H47 * Q^n)
        curObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

        // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
        curObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * this->kinematic[seg_idx].F_Omg_inSeg);

        // Update
        rollingQAdd = rollingQAdd + rollingQFac;
        rollingQFac = rollingQFac * this->kinematic[seg_idx].F_Q_inSeg;
    }
}

void Observability::batchStripObsMat_Frame(const int start_idx, const int end_idx) {

    assert(start_idx < end_idx);
    //
    for(int i = start_idx; i < end_idx; i ++)  {
        //        std::cout << "lmk " << i << std::endl;
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (this->pFrame->mvbOutlier[i] == false &&
                    this->pFrame->mvbCandidate[i] == true) {

                // skip if the Jacobian has already been built
                // if (this->pFrame->mvbJacobBuilt[i] == true)
                //     continue ;

                if (pMP->updateAtFrameId == this->mnFrameId)
                    continue ;

                // Feature position
                cv::Mat Pw = pMP->GetWorldPos();
                arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                Y[0] = Pw.at<float>(0);
                Y[1] = Pw.at<float>(1);
                Y[2] = Pw.at<float>(2);

                arma::mat TOM;
                arma::mat F_fac(13, 13);
                F_fac.eye();
                //
                size_t j;
                for (j=0; j<this->kinematic.size(); ++j) {
                    // Get the position of measurement
                    arma::rowvec Z = arma::zeros<arma::rowvec>(2);
                    if (j == 0) {
                        // use the actual measurement
                        cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                        Z[0] = kpUn.pt.x;
                        Z[1] = kpUn.pt.y;
                    }
                    else {
                        // predict the measurement
                        //
                        float u, v;
                        arma::mat proj_jacob;
                        project_Point_To_Frame(Pw, this->kinematic[j].Tcw, u, v, proj_jacob);
                        Z[0] = u;
                        Z[1] = v;
                    }
                    //                    std::cout << Z << std::endl;

                    // terminate if went out of bound
                    if (Z[0] < 0 || Z[1] < 0) {
                        break ;
                    }
                    //                    std::cout << Z << std::endl;

                    // Compute the In-segment Observability Matrix
                    arma::mat curSOM;
                    compute_SOM_In_Segment(j, Y, Z, curSOM);

                    // Multiply with full-seg system matrix, so as to
                    // obtain total obs matrix
                    TOM = arma::join_vert(TOM, curSOM * F_fac);
                    F_fac = this->kinematic[j].F * F_fac;
                }

                if (TOM.n_rows > 0) {
                    pMP->ObsMat  = TOM;
                    //                pMP->ObsRank = arma::rank(TOM);
                    pMP->ObsScore = 1.0;
                }
                else {
                    pMP->ObsScore = -1.0;
                }

                //
                // this->pFrame->mvbJacobBuilt[i] = true;
                pMP->updateAtFrameId = this->mnFrameId;

            } //Perform regular temporal obs update!
        }
    } // For: all pMP
}

void Observability::batchHybridMat_Frame(const size_t start_idx, const size_t end_idx, const double time_for_build) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_proj, H_rw;
    float res_u = 0, res_v = 0, u, v;
    arma::mat ZMat;
    ZMat.zeros(2, 6);

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->pFrame->mvpMapPoints.size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            std::cout << "func batchHybridMat_Frame: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->pFrame->mvpMapPoints[i];
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {
            if (this->pFrame->mvbOutlier[i] == false &&
                    this->pFrame->mvbCandidate[i] == true) {

                // skip if the Jacobian has already been built
                //  if (this->pFrame->mvbJacobBuilt[i] == true)
                //      continue ;

                if (pMP->updateAtFrameId == this->mnFrameId)
                    continue ;

                // Feature position
                arma::rowvec Y = arma::zeros<arma::rowvec>(4);
                cv::Mat Pw = pMP->GetWorldPos();
                Y[0] = Pw.at<float>(0);
                Y[1] = Pw.at<float>(1);
                Y[2] = Pw.at<float>(2);
                Y[3] = 1;

                // Measurement
                arma::rowvec Z = arma::zeros<arma::rowvec>(2);
                cv::KeyPoint kpUn = this->pFrame->mvKeysUn[i];
                Z[0] = kpUn.pt.x;
                Z[1] = kpUn.pt.y;

#if defined LOCAL_JACOBIAN

                //                std::cout << "Tcw_prev = " << this->Tcw_prev << std::endl;
                //                std::cout << "Twc = " << this->Twc << std::endl;

                // convert Y into local version
                // Y_rel
                arma::mat Y_rel = this->Tcw_prev * Y;
                //                std::cout << "Y_rel = " << Y_rel << std::endl;
                //
                compute_H_subblock_fast(this->Xv_rel, Y_rel.cols(0, 2), Z, H13, H47, res_u, res_v);
                //                std::cout << "H13 = " << H13 << std::endl;
                //                std::cout << "H47 = " << H47 << std::endl;
#else
                //            timer.tic();
                // Compute the H matrix
                //            compute_H_subblock(this->Xv, Y, Z, H13, H47);
                // compute_H_subblock_fast(this->kinematic[0].Xv, Y.cols(0, 2), Z, H13, H47);
                //                compute_H_subblock_complete(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), Z, H13, H47, H_proj, res_u, res_v);
                compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H13, H47, H_proj, false, u, v);
                res_u = Z[0] - u;
                res_v = Z[1] - v;
                //            time_H += timer.toc();

                //                std::cout << H13 << " " << H47 << std::endl;
#endif

                // assemble into H matrix
                H_meas = arma::join_horiz(H13, H47);
                H_meas = arma::join_horiz(H_meas, ZMat);

                reWeightInfoMat( this->pFrame, i, pMP, H_meas, res_u, res_v, H_proj, H_rw );

                arma::mat Hyb = arma::join_vert(H_rw, H_rw * this->kinematic[this->mKineIdx].F);
                //                arma::mat HF = H_rw * this->kinematic[this->mKineIdx].F;
                //                arma::mat HF_2 = HF * this->kinematic[this->mKineIdx].F;
                //                arma::mat HF_3 = HF_2 * this->kinematic[this->mKineIdx].F;
                //                //
                //                arma::mat Hyb = arma::join_vert(H_rw, HF, HF_2, HF_3);
                arma::mat infMat = Hyb.t() * Hyb;
                //                std::cout << "H_rw = " << H_rw << std::endl;
                //                std::cout << "F = " << this->kinematic[this->mKineIdx].F << std::endl;
                //                std::cout << "Hyb = " << Hyb << std::endl;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->u_proj = u;
                pMP->v_proj = v;
                pMP->H_meas = H_meas;
                pMP->H_proj = H_proj;
                pMP->ObsMat = infMat;
                pMP->updateAtFrameId = this->mnFrameId;

                // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, 2);
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, "fro");
                //                pMP->ObsScore = 1.0;
                pMP->ObsScore = this->pFrame->mvpMatchScore[i];
                //
                //  this->pFrame->mvbJacobBuilt[i] = true;
                pMP->updateAtFrameId = this->mnFrameId;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
            else {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
        } //Perform regular temporal obs update!
    } // For: all pMP
}

void Observability::batchHybridMat_Map(const size_t start_idx, const size_t end_idx, const double time_for_build, const bool check_viz) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_proj, H_rw;
    float u, v;
    arma::mat ZMat;
    ZMat.zeros(2, 6);

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->mMapPoints->size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            std::cout << "func batchHybridMat_Map: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->mMapPoints->at(i);
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {

            if (pMP->updateAtFrameId == this->mnFrameId)
                continue ;

            if (check_viz == false && pMP->mbTrackInView == false)
                continue ;

            // Feature position
            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
            cv::Mat featPos = pMP->GetWorldPos();
            Y[0] = featPos.at<float>(0);
            Y[1] = featPos.at<float>(1);
            Y[2] = featPos.at<float>(2);

            bool flag = compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y, H13, H47, H_proj, check_viz, u, v);
            if (flag == false) {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;
                //                pMP->updateAtFrameId = this->mnFrameId;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
                continue ;
            }

            //            std::cout << "find visible lmk " << i << std::endl;

            // assemble into H matrix
            H_meas = arma::join_horiz(H13, H47);
            H_meas = arma::join_horiz(H_meas, ZMat);

            reWeightInfoMat( NULL, -1, pMP, H_meas, 0, 0, H_proj, H_rw );

            arma::mat Hyb = arma::join_vert(H_rw, H_rw * this->kinematic[this->mKineIdx].F);
            //            arma::mat HF = H_rw * this->kinematic[this->mKineIdx].F;
            //            arma::mat HF_2 = HF * this->kinematic[this->mKineIdx].F;
            //            arma::mat HF_3 = HF_2 * this->kinematic[this->mKineIdx].F;
            //            //
            //            arma::mat Hyb = arma::join_vert(H_rw, HF, HF_2, HF_3);
            arma::mat infMat = Hyb.t() * Hyb;

#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif

            pMP->u_proj = u;
            pMP->v_proj = v;
            pMP->H_meas = H_meas;
            pMP->H_proj = H_proj;
            pMP->ObsMat = infMat;
            // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
            pMP->ObsScore = 1.0;
            pMP->updateAtFrameId = this->mnFrameId;
            //            lmkSelectPool.push_back( tmpLmk );

#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
    } //Perform regular temporal obs update!
}

void Observability::batchInfoMat_Frame(const size_t start_idx, const size_t end_idx, const double time_for_build) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_proj, H_disp, H_rw;
    float res_u = 0, res_v = 0, u, v;

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->pFrame->mvpMapPoints.size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            std::cout << "func batchInfoMat_Frame: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->pFrame->mvpMapPoints[i];
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {
            if (this->pFrame->mvbOutlier[i] == false &&
                    this->pFrame->mvbCandidate[i] == true) {

                if (pMP->updateAtFrameId == this->mnFrameId)
                    continue ;

                // Feature position
                arma::rowvec Y = arma::zeros<arma::rowvec>(4);
                cv::Mat Pw = pMP->GetWorldPos();
                Y[0] = Pw.at<float>(0);
                Y[1] = Pw.at<float>(1);
                Y[2] = Pw.at<float>(2);
                Y[3] = 1;

                // Measurement
                arma::rowvec Z = arma::zeros<arma::rowvec>(2);
                cv::KeyPoint kpUn = this->pFrame->mvKeysUn[i];
                Z[0] = kpUn.pt.x;
                Z[1] = kpUn.pt.y;

#if defined LOCAL_JACOBIAN

                //                std::cout << "Tcw_prev = " << this->Tcw_prev << std::endl;
                //                std::cout << "Twc = " << this->Twc << std::endl;

                // convert Y into local version
                // Y_rel
                arma::mat Y_rel = this->Tcw_prev * Y;
                //                std::cout << "Y_rel = " << Y_rel << std::endl;
                //
                compute_H_subblock_fast(this->Xv_rel, Y_rel.cols(0, 2), Z, H13, H47, res_u, res_v);
                //                std::cout << "H13 = " << H13 << std::endl;
                //                std::cout << "H47 = " << H47 << std::endl;
#else
                //            timer.tic();
                // Compute the H matrix
                //            compute_H_subblock(this->Xv, Y, Z, H13, H47);
                // compute_H_subblock_fast(this->kinematic[0].Xv, Y.cols(0, 2), Z, H13, H47);
                //                compute_H_subblock_complete(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), Z, H13, H47, H_proj, res_u, res_v);
                compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H13, H47, H_proj, false, u, v);
                res_u = Z[0] - u;
                res_v = Z[1] - v;
                //            time_H += timer.toc();

                //                std::cout << H13 << " " << H47 << std::endl;
#endif

                // assemble into H matrix
                H_meas = arma::join_horiz(H13, H47);

                //#ifdef DELAYED_STEREO_MATCHING
                if (mSensor == 1 || mSensor == 2) {
                    if (this->pFrame->mvuRight[i] >= 0) {
                        // add the disparity info term to curMat
                        compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H_disp);
                        H_meas = arma::join_vert(H_meas, H_disp);
                    }
                }
                //                cout << H_meas << endl << endl;
                //#endif

                reWeightInfoMat( this->pFrame, i, pMP, H_meas, res_u, res_v, H_proj, H_rw );
                arma::mat infMat = H_rw.t() * H_rw;
                //                GoodPoint tmpLmk(static_cast<size_t>(i), this->pFrame->mvpMatchScore[i], infMat);

#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif
                //                // Update the total information matrix
                //                int num_row_obsMat = static_cast<int>(pMP->ObsMat.n_rows);
                //                arma::mat ObsMat_tmp = pMP->ObsMat;

                //                if (num_row_obsMat >= 2*OBS_SEGMENT_NUM) {
                //                    // remove the 2 rows in the bottom
                //                    ObsMat_tmp.shed_rows(2*OBS_SEGMENT_NUM-2, num_row_obsMat-1);
                //                }

                //                // multiply the rest blocks by F matrix
                //                if (ObsMat_tmp.n_rows > 0) {
                //                    ObsMat_tmp = ObsMat_tmp * this->kinematic[0].F;
                //                }

                //                // add new block to the head of SOM
                //                pMP->ObsMat  = arma::join_vert(H, ObsMat_tmp);
                //                //                std::cout << pMP->ObsMat << std::endl;
                pMP->u_proj = u;
                pMP->v_proj = v;
                pMP->H_meas = H_meas;
                pMP->H_proj = H_proj;
                pMP->ObsMat = infMat;

                // TODO
                // for some reason, adding all frame-by-frame matches into the prior info matrix hurts the performance of good feature;
                // simply ignore all of them also leads to degeneracy.
                // the best performance is obtained when only the prior information from previous map is used while the current frame-by-frame ones are ignored
                // later we need to dig into this issue.
                pMP->updateAtFrameId = this->mnFrameId;

                // compute observability score
#if defined OBS_MIN_SVD
                arma::vec s = arma::svd(H);
                pMP->ObsScore = s(12);
#elif defined OBS_MAX_VOL || defined OBS_MAXVOL_TOP_SV
                double s;
                arma::colvec u;
                arma::rowvec v;
                solveMaxSVD(H, u, s, v);
                pMP->ObsVector = arma::vec(DIMENSION_OF_STATE_MODEL);
                for (size_t dn=0; dn < DIMENSION_OF_STATE_MODEL; ++ dn) {
                    pMP->ObsVector[dn] = v(dn);
                }
                pMP->ObsScore = s;
#elif defined OBS_MAXVOL_FULL || defined OBS_MAXVOL_Greedy
                // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, 2);
                //                pMP->ObsScore = arma::norm(pMP->ObsMat, "fro");
                //                pMP->ObsScore = 1.0;
                pMP->ObsScore = this->pFrame->mvpMatchScore[i];
#endif
                //

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
            else {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
            }
        } //Perform regular temporal obs update!
    } // For: all pMP

    //#ifdef OBS_DEBUG_VERBOSE
    //    std::cout << "func getObsMatrix: Time cost of H submatrix = " << time_H << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of Build Inst SOM = " << time_BuildMat << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of Update SOM = " << time_UpdateMat << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of arma::svd = " << time_SVD_1 << std::endl;
    //    std::cout << "func getObsMatrix: Time cost of max-vector svd = " << time_SVD_2 << std::endl;
    //#endif
}

void Observability::batchInfoMat_Map(const size_t start_idx, const size_t end_idx, const double time_for_build, const bool check_viz) {

    if (this->mKineIdx >= this->kinematic.size())
        return ;

    arma::mat H13, H47, H_meas, H_disp, H_proj, H_rw;
    float u, v;

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    for(size_t i = start_idx; i < end_idx; i++)  {
        //
        if (i >= this->mMapPoints->size())
            break ;

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_build) {
            //            std::cout << "func batchInfoMat_Map: reach max time cap " << timeCap << std::endl;
            break ;
        }

        //        std::cout << i << std::endl;
        MapPoint* pMP = this->mMapPoints->at(i);
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {

            if (pMP->updateAtFrameId == this->mnFrameId)
                continue ;

            if (check_viz == false && pMP->mbTrackInView == false)
                continue ;

            // Feature position
            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
            cv::Mat featPos = pMP->GetWorldPos();
            Y[0] = featPos.at<float>(0);
            Y[1] = featPos.at<float>(1);
            Y[2] = featPos.at<float>(2);

            bool flag = compute_H_subblock_simplied(this->kinematic[this->mKineIdx].Xv, Y, H13, H47, H_proj, check_viz, u, v);
            if (flag == false) {
#ifdef MULTI_THREAD_LOCK_ON
                mtx.lock();
#endif

                pMP->ObsScore = -1.0;
                //                pMP->updateAtFrameId = this->mnFrameId;

#ifdef MULTI_THREAD_LOCK_ON
                mtx.unlock();
#endif
                continue ;
            }

            //            std::cout << "find visible lmk " << i << std::endl;

            // assemble into H matrix
            H_meas = arma::join_horiz(H13, H47);

            // NOTE
            // for RGBD data, assume the depth / disparity will be measured
            // therefore including the addition row in measurement Jacobian
            // meanwhile for mono & stereo setup, assuming only the left measurement being avaible
            if (mSensor == 2) {
                // add the disparity info term to curMat
                compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y.subvec(0, 2), H_disp);
                H_meas = arma::join_vert(H_meas, H_disp);
            }

            arma::mat infMat = H_meas.t() * H_meas;
            //            reWeightInfoMat( NULL, -1, pMP, H_meas, 0, 0, H_proj, H_rw );
            //            arma::mat infMat = H_rw.t() * H_rw;

#ifdef MULTI_THREAD_LOCK_ON
            mtx.lock();
#endif

            pMP->u_proj = u;
            pMP->v_proj = v;
            pMP->H_meas = H_meas;
            pMP->H_proj = H_proj;
            pMP->ObsMat = infMat;
            // do nothing on the obs matrix; set the score to 1 for up-coming subset selection
            pMP->ObsScore = 1.0;

            // TO DO
            pMP->updateAtFrameId = this->mnFrameId;
            //            lmkSelectPool.push_back( tmpLmk );

#ifdef MULTI_THREAD_LOCK_ON
            mtx.unlock();
#endif
        }
    } //Perform regular temporal obs update!
}

bool Observability::runMatrixBuilding(const size_t mat_type, const double time_for_build,
                                      const bool with_multi_thread, const bool check_viz) {

    int N = 0;

    switch (mat_type) {
    case ORB_SLAM2::MAP_INFO_MATRIX:
        //
        if (this->mMapPoints == NULL)
            return false;

        N = this->mMapPoints->size();

        if (with_multi_thread) {

            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchInfoMat_Map, this, i*grainSize, (i+1)*grainSize, time_for_build, check_viz);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchInfoMat_Map((mNumThreads-1)*grainSize, N, time_for_build, check_viz);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }

            delete [] mThreads;

        }
        else {

            // single-thread matrix construction
            this->batchInfoMat_Map(0, N, time_for_build, check_viz);

        }

        break;

    case ORB_SLAM2::FRAME_INFO_MATRIX:
        //
        if (this->pFrame == NULL)
            return false;

        N = this->pFrame->mvpMapPoints.size();

        if (with_multi_thread) {

            // original solution, utilizing c++ 11 std thread
            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchInfoMat_Frame, this, i*grainSize, (i+1)*grainSize, time_for_build);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchInfoMat_Frame((mNumThreads-1)*grainSize, N, time_for_build);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }
            delete [] mThreads;
            //        cout << "done with info mat multi thread!" << endl;

        }
        else {

            // single-thread matrix construction
            this->batchInfoMat_Frame(0, N, time_for_build);

        }

        break;

    case ORB_SLAM2::MAP_HYBRID_MATRIX:
        //
        if (this->mMapPoints == NULL)
            return false;

        N = this->mMapPoints->size();

        if (with_multi_thread) {

            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchHybridMat_Map, this, i*grainSize, (i+1)*grainSize, time_for_build, check_viz);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchHybridMat_Map((mNumThreads-1)*grainSize, N, time_for_build, check_viz);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }

            delete [] mThreads;

        }
        else {

            // single-thread matrix construction
            this->batchHybridMat_Map(0, N, time_for_build, check_viz);

        }

        break;

    case ORB_SLAM2::FRAME_HYBRID_MATRIX:
        //
        if (this->pFrame == NULL)
            return false;

        N = this->pFrame->mvpMapPoints.size();

        if (with_multi_thread) {

            // original solution, utilizing c++ 11 std thread
            int grainSize = static_cast<double>(N)/static_cast<double>(mNumThreads);

            if (mNumThreads > 1)
                mThreads = new std::thread [mNumThreads-1];
            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i] = std::thread(&Observability::batchHybridMat_Frame, this, i*grainSize, (i+1)*grainSize, time_for_build);
            }
            //    mThreads[mNumThreads-1] = std::thread(&Observability::batchBuildInstInfoMat, this, (mNumThreads-1)*grainSize, N);
            this->batchHybridMat_Frame((mNumThreads-1)*grainSize, N, time_for_build);

            for (size_t i = 0; i < mNumThreads-1; i++) {
                mThreads[i].join();
            }
            delete [] mThreads;
            //        cout << "done with info mat multi thread!" << endl;

        }
        else {

            // single-thread matrix construction
            this->batchHybridMat_Frame(0, N, time_for_build);

        }

        break;

    case ORB_SLAM2::FRAME_STRIP_OBS_MATRIX:
        // TODO
        break;

    default:
        // do nothing
        cerr << "unknown matrix type!" << endl;
    }

    return true;
}

// ==================================================================================

int Observability::runActiveMapMatching(Frame *pFrame,
                                        const size_t mat_type,
                                        const arma::mat &mBaseInfoMat,
                                        const float th,
                                        ORBmatcher &mORBMatcher,
                                        const int num_to_match,
                                        const double time_for_match ) {

    //    mLeftMapPoints.clear();
    if (pFrame == NULL || this->mMapPoints == NULL)
        return 0;
    if (mMapPoints->size() == 0 || num_to_match <= 0 || time_for_match <= 0) {
        //
        for (size_t i=0; i<mMapPoints->size(); ++i) {
            if (mMapPoints->at(i) == NULL)
                continue ;
            if (mMapPoints->at(i)->isBad())
                continue ;
            if (mMapPoints->at(i)->mbTrackInView == false)
                continue;
            //
            mLeftMapPoints.push_back(mMapPoints->at(i));
        }
        return 0;
    }

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    // iteratively search for the most informative lmk
    arma::mat curMat = mBaseInfoMat;
    arma::mat H_disp;
    int thStereo = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;
    double curCost = 0;

    // create a query index for fast insert and reject of entries
    arma::mat lmkIdx = arma::mat(1, mMapPoints->size());
    // create a flag array to avoid duplicate visit
    arma::mat lmkVisited = arma::mat(1, mMapPoints->size());
    size_t N = 0;
    for (size_t i=0; i<mMapPoints->size(); ++i) {
        if (mMapPoints->at(i) == NULL)
            continue ;
        if (mMapPoints->at(i)->isBad())
            continue ;
        if (mMapPoints->at(i)->mbTrackInView == false)
            continue;
        if (mMapPoints->at(i)->updateAtFrameId != pFrame->mnId)
            continue ;

        //
#ifdef INFORMATION_EFFICIENCY_SCORE
        mORBMatcher.GetCandidates(*pFrame, mMapPoints->at(i), th);
        if (mMapPoints->at(i)->mvMatchCandidates.empty())
            continue ;
#endif

        lmkIdx.at(0, N) = i;
        lmkVisited.at(0, N) = -1;
        ++ N;
    }
    lmkIdx.resize(1, N);
    lmkVisited.resize(1, N);
    //    std::cout << "func runActiveMapMatching: start active matching " << num_to_match
    //              << " lmks from " << N << " visible map points!" << std::endl;

    // NOTE
    // define the size of random subset
    // in practice, multiplication factor 1.0 works better on many sequences, 
    // mostly because it is doing feature matching rather than selection:
    // matching ratio would affect the random subset size here!
    size_t szLazierSubset = static_cast<size_t>( float(N) / float(num_to_match) * 1.0 );
    // in theory, multiplication factor 2.3 is equivalant to decay factor of 0.1
    // size_t szLazierSubset = static_cast<size_t>( float(N) / float(num_to_match) * 2.3 );

    int nMatched = 0;
    //
    while (nMatched < num_to_match) {
        //    for (size_t i=0; i<num_to_match; ++i) {
        //        int maxLmk = -1;
        //        double maxDet = -DBL_MAX;
        std::priority_queue<SimplePoint> heapSubset;
        std::vector<size_t> removeIdx;
#ifdef OBS_DEBUG_VERBOSE
        std::cout << "func matchMapPointActive: matching round " << nMatched << std::endl;
#endif
        size_t numHit = 0, numRndQue = 0, szActualSubset;
        szActualSubset = szLazierSubset;
        if (lmkIdx.n_cols < szActualSubset)
            szActualSubset = lmkIdx.n_cols;
        //
        //        std::srand(std::time(nullptr));
        while (numHit < szActualSubset) {
            // generate random query index
#ifdef OBS_DEBUG_VERBOSE
            cout << "random query round " << numHit << endl;
#endif
            size_t j;
            numRndQue = 0;
            while (numRndQue < MAX_RANDOM_QUERY_TIME) {
                j = ( std::rand() % lmkIdx.n_cols );
                //                cout << j << " ";
                // check if visited
                if (lmkVisited.at(0, j) < nMatched) {
                    lmkVisited.at(0, j) = nMatched;
                    break ;
                }
                ++ numRndQue;
            }
            if (numRndQue >= MAX_RANDOM_QUERY_TIME) {
                //                cout << "Failed to find a valid random map point to start with!" << endl;
                break ;
            }

            int queIdx = lmkIdx.at(0, j);
            ++ numHit;

            // apply cap on time cost
            timeCap = timer.toc();
            if (timeCap > time_for_match) {
                cout << "reach max time cap for active matching!" << timeCap << endl;
                return nMatched;
            }

            // else, the queried map point is updated in time, and being assessed with potential info gain
            double curDet = logDet( curMat + mMapPoints->at(queIdx)->ObsMat );
#ifdef INFORMATION_EFFICIENCY_SCORE
            heapSubset.push(SimplePoint(queIdx, curDet / double(curCost + mMapPoints->at(queIdx)->mvMatchCandidates.size())));
#else
            heapSubset.push(SimplePoint(queIdx, curDet));
#endif

            if (numHit >= szActualSubset) {
                // once filled the heap with random samples
                // start matching the one with highest logdet / info gain
                SimplePoint heapTop = heapSubset.top();
#ifdef OBS_DEBUG_VERBOSE
                cout << "heapTop logDet = " << heapTop.score << endl;
#endif

#ifdef INFORMATION_EFFICIENCY_SCORE
                int bestIdx = mORBMatcher.MatchCandidates(*pFrame, mMapPoints->at(heapTop.idx));
                curCost += mMapPoints->at(heapTop.idx)->mvMatchCandidates.size();
#else
                int bestIdx = mORBMatcher.SearchByProjection_OnePoint(*pFrame, mMapPoints->at(heapTop.idx), th);
#endif
                if (bestIdx >= 0) {
                    //                    std::cout << "Found match " << heapSubset.top().idx << std::endl;
                    // match succeed, move on to next batch random eval
                    //                    curMat = curMat + mMapPoints->at(heapTop.idx)->ObsMat;

                    if (mSensor == 1) {
#ifdef DELAYED_STEREO_MATCHING
                        //
                        cv::Mat Pw = mMapPoints->at(heapTop.idx)->GetWorldPos(), Pc;
                        //                        cout << "Pw = " << Pw.at<float>(0) << ", " << Pw.at<float>(1) << ", " << Pw.at<float>(2) << endl;
                        if (pFrame->WorldToCameraPoint(Pw, Pc) == true) {
                            //                            cout << "Pc = " << Pc.at<float>(0) << ", " << Pc.at<float>(1) << ", " << Pc.at<float>(2) << endl;
                            // check the range of disparity
                            float disp = float(pFrame->mbf) / Pc.at<float>(2);
                            float disp_min = std::max(disp - float(DISPARITY_THRES), 0.0f),
                                    disp_max = std::min(disp + float(DISPARITY_THRES), float(pFrame->mbf)/float(pFrame->mb));

                            // stereo matching with the narrowed disparity range
                            if (pFrame->ComputeStereoMatch_OnePoint(bestIdx, thStereo, disp_min, disp_max) == true) {
                                // grab the info term from right cam stereo match
                                // feature position
                                arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                                cv::Mat featPos = mMapPoints->at(heapTop.idx)->GetWorldPos();
                                Y[0] = featPos.at<float>(0);
                                Y[1] = featPos.at<float>(1);
                                Y[2] = featPos.at<float>(2);
                                // add the disparity info term to curMat
                                compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y, H_disp);
                                //                        curMat = curMat + H_disp.t() * H_disp;
                                mMapPoints->at(heapTop.idx)->H_meas = arma::join_vert(mMapPoints->at(heapTop.idx)->H_meas, H_disp);

                                nMatched += 1;
                            }
                        }
#else
                        // grab the info term from right cam stereo match feature position
                        if (pFrame->mvDepth[bestIdx] >= 0) {
                            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                            cv::Mat featPos = mMapPoints->at(heapTop.idx)->GetWorldPos();
                            Y[0] = featPos.at<float>(0);
                            Y[1] = featPos.at<float>(1);
                            Y[2] = featPos.at<float>(2);
                            // add the disparity info term to curMat
                            compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y, H_disp);
                            mMapPoints->at(heapTop.idx)->H_meas = arma::join_vert(mMapPoints->at(heapTop.idx)->H_meas, H_disp);

                            nMatched += 1;
                        }
#endif
                    }
                    else if (mSensor == 2) {
                        // grab the info term from depth cam
                        if (pFrame->mvDepth[bestIdx] >= 0) {
                            arma::rowvec Y = arma::zeros<arma::rowvec>(3);
                            cv::Mat featPos = mMapPoints->at(heapTop.idx)->GetWorldPos();
                            Y[0] = featPos.at<float>(0);
                            Y[1] = featPos.at<float>(1);
                            Y[2] = featPos.at<float>(2);
                            // add the disparity info term to curMat
                            compute_H_disparity_col (this->kinematic[this->mKineIdx].Xv, Y, H_disp);
                            mMapPoints->at(heapTop.idx)->H_meas = arma::join_vert(mMapPoints->at(heapTop.idx)->H_meas, H_disp);

                            nMatched += 1;
                        }
                    }

                    // TODO
                    // update the info matrix with measurement info, e.g. ocl level, residual
                    float res_u = pFrame->mvKeysUn[bestIdx].pt.x - mMapPoints->at(heapTop.idx)->u_proj,
                            res_v = pFrame->mvKeysUn[bestIdx].pt.y - mMapPoints->at(heapTop.idx)->v_proj;
                    //                    pFrame->getProjectError(pFrame->mvpMapPoints[bestIdx], &(pFrame->mvKeysUn[bestIdx]), res_u, res_v);

                    arma::mat H_rw;
                    reWeightInfoMat( pFrame, bestIdx, mMapPoints->at(heapTop.idx),
                                     mMapPoints->at(heapTop.idx)->H_meas, res_u, res_v,
                                     mMapPoints->at(heapTop.idx)->H_proj, H_rw );
                    //                    std::cout << "H_rw = "  << H_rw << std::endl;

                    if (mat_type == ORB_SLAM2::FRAME_HYBRID_MATRIX || mat_type == ORB_SLAM2::MAP_HYBRID_MATRIX) {
                        //
                        arma::mat Hyb = arma::join_vert(H_rw, H_rw * this->kinematic[this->mKineIdx].F);
                        curMat = curMat + Hyb.t() * Hyb;
                    }
                    else if (mat_type == ORB_SLAM2::FRAME_INFO_MATRIX || mat_type == ORB_SLAM2::MAP_INFO_MATRIX) {
                        //
                        curMat = curMat + H_rw.t() * H_rw;
                    }
                    else {
                        // TODO
                    }

                    removeIdx.push_back(heapTop.idx);
                    nMatched += 2;
                    break ;
                }
                else {
                    // otherwise, remove the top one from map points, and re-sample a map points
                    //                    std::cout << "Failed to match " << heapSubset.top().idx << std::endl;
                    removeIdx.push_back(heapTop.idx);
                    heapSubset.pop();
                    -- numHit;
                }
            }
        }

        if (numRndQue >= MAX_RANDOM_QUERY_TIME || heapSubset.size() == 0 || removeIdx.size() == 0) {
            std::cout << "func runActiveMapMatching: early termination!" << std::endl;
            break ;
        }

        //        cout << "heap check: ";
        //        for (int j=0; j<heapSubset.size(); ++j) {
        //            cout << heapSubset.top().score << " ";
        //            heapSubset.pop();
        //        }
        //        cout << endl;

        if (lmkIdx.n_cols == removeIdx.size()) {
            std::cout << "func runActiveMapMatching: went through all map points!" << std::endl;
            break ;
        }

        // set up the index for columns that are not selected yet
        std::sort(removeIdx.begin(), removeIdx.end());
        arma::uvec restCol = arma::uvec(lmkIdx.n_cols-removeIdx.size());

#ifdef OBS_DEBUG_VERBOSE
        cout << "before clean entries!" << endl;
        cout << "removeIdx: " << endl;
        for (int j=0; j<removeIdx.size(); ++j)
            cout << removeIdx[j] << " " ;
        cout << endl;
        //        cout << "lmkIdx: " << endl;
        //        for (int j=0; j<lmkIdx.n_cols; ++j)
        //            cout << lmkIdx.at(0,j) << " " ;
        //        cout << endl;
        cout << "restCol: " << endl;
        cout << "size = " << lmkIdx.n_cols-removeIdx.size() << endl;
#endif

        size_t j = 0, k = 0, l = 0;
        while (j < lmkIdx.n_cols) {
            if (k >= removeIdx.size()) {
                restCol[l] = j;
                ++l;
                ++j;
            }
            else {
                if (lmkIdx.at(0,j) < removeIdx[k]) {
                    restCol[l] = j;
                    ++l;
                    ++j;
                }
                else {
                    if (lmkIdx.at(0,j) == removeIdx[k] ) {
                        ++j;
                    }
                    ++k;
                }
            }
        }
        lmkIdx = lmkIdx.cols(restCol);
        lmkVisited = lmkVisited.cols(restCol);

#ifdef OBS_DEBUG_VERBOSE
        cout << "after clean entries!" << endl;
        cout << "lmkIdx.n_cols = " << lmkIdx.n_cols << endl;
#endif
        //        // apply cap on time cost
        //        timeCap = timer.toc();
        //        if (timeCap > MAX_TIMECOST_SELECT) {
        //            //            cout << "reach max time cap for current greedy!" << timeCap << endl;
        //            //            break ;
        //            return false;
        //        }
    }

    //    std::cout << "func cutMaxVolSubset: time cost of assemble X" << time_Assemb
    //              << ", column norm = " << time_CNorm << ", append result = " << time_Append
    //              << ", set new cindex = " << time_Index << ", project column (appr) = " << time_Proj
    //              << ", remove column = " << time_Remov << std::endl;
    //#ifdef RANDOM_SHUFFLE_LAZIER_GREEDY
    //    delete rndEntry;
    //#endif
    //    mLeftMapPoints.clear();
    for (size_t i=0; i<lmkIdx.n_cols; ++i) {
        mLeftMapPoints.push_back(mMapPoints->at(lmkIdx.at(0, i)));
    }

    //    std::cout << "func matchMapPointActive: done with " << num_to_match << " iterations!" << std::endl;
    return nMatched;

}

int Observability::runBaselineMapMatching(Frame *pFrame,
                                          const size_t base_mtd,
                                          const float th,
                                          ORBmatcher &mORBMatcher,
                                          const int num_to_match,
                                          const double time_for_match ) {

    //    mLeftMapPoints.clear();
    if (pFrame == NULL || this->mMapPoints == NULL)
        return 0;
    if (mMapPoints->size() == 0 || num_to_match <= 0 || time_for_match <= 0) {
        //
        for (size_t i=0; i<mMapPoints->size(); ++i) {
            if (mMapPoints->at(i) == NULL)
                continue ;
            if (mMapPoints->at(i)->isBad())
                continue ;
            if (mMapPoints->at(i)->mbTrackInView == false)
                continue;
            //
            mLeftMapPoints.push_back(mMapPoints->at(i));
        }
        return 0;
    }

    double timeCap = 0;
    arma::wall_clock timer;
    timer.tic();

    int nMatched = 0;
    vector<GoodPoint> vldPoints;

    for (size_t i=0; i<mMapPoints->size(); ++i) {
        if (mMapPoints->at(i) == NULL)
            continue ;
        if (mMapPoints->at(i)->isBad())
            continue ;
        if (mMapPoints->at(i)->mbTrackInView == false)
            continue;
        //
        vldPoints.push_back(GoodPoint(i, mMapPoints->at(i)->mnVisible));
    }
    std::cout << "func runBaselineMapMatching: start active matching " << num_to_match
              << " lmks from " << vldPoints.size() << " visible map points!" << std::endl;

    //
    if (base_mtd == ORB_SLAM2::BASELINE_RANDOM) {
        // random shuffle
        std::random_shuffle ( vldPoints.begin(), vldPoints.end() );
    }
    else if (base_mtd == ORB_SLAM2::BASELINE_LONGLIVE) {
        // sort by life
        std::sort(vldPoints.begin(), vldPoints.end(), GoodPoint::rankObsScore_descend);
    }
    else {
        //
        std::cerr << "unknown baseline method being called!" << std::endl;
    }

    //
    int thStereo = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;
    size_t i;
    for (i=0; i<vldPoints.size(); ++i) {

        // apply cap on time cost
        timeCap = timer.toc();
        if (timeCap > time_for_match) {
            cout << "reach max time cap for active matching!" << timeCap << endl;
            return nMatched;
        }

        if (nMatched >= num_to_match)
            break ;

        int bestIdx = mORBMatcher.SearchByProjection_OnePoint(*pFrame, mMapPoints->at(vldPoints[i].idx), th);
        if (bestIdx >= 0) {
            nMatched += 2;

            if (mSensor == 1) {
#ifdef DELAYED_STEREO_MATCHING
                //
                cv::Mat Pw = mMapPoints->at(vldPoints[i].idx)->GetWorldPos(), Pc;
                if (pFrame->WorldToCameraPoint(Pw, Pc) == true) {
                    // check the range of disparity
                    float disp = float(pFrame->mbf) / Pc.at<float>(2);
                    float disp_min = std::max(disp - float(DISPARITY_THRES), 0.0f),
                            disp_max = std::min(disp + float(DISPARITY_THRES), float(pFrame->mbf)/float(pFrame->mb));
                    // stereo matching with the narrowed disparity range
                    pFrame->ComputeStereoMatch_OnePoint(bestIdx, thStereo, disp_min, disp_max);
                }
#endif
            }

            if (pFrame->mvDepth[bestIdx] >= 0)
                nMatched += 1;
        }
    }

    //    mLeftMapPoints.clear();
    while (i<vldPoints.size()) {
        mLeftMapPoints.push_back(mMapPoints->at(vldPoints[i].idx));
        ++i;
    }

    //    std::cout << "func matchMapPointActive: done with " << num_to_match << " iterations!" << std::endl;
    return nMatched;

}

}
