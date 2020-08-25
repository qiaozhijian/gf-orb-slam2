#pragma once

#include <iostream>     // std::cout, std::fixed
#include <iomanip>
#include <stdio.h> // printf
#include <fstream>
#include <string>
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>

//#include <Eigen/StdVector>
#include "good_graph_testbed/BAOptimizer.h" // BA types

using namespace std;

#define WORLD_RANGE         40.0 // 150.0 //
#define MAX_VISIBLE_DIST    100.0 // 10.0 // 30.0 //

#define T_STEP      5.0 // 2.0
#define E_STEP      0.5 // 0.8

// random perturbation added to camera pose
#define PT_RANGE  0.10 // 0.30 //
#define PA_RANGE  5.0 / 180.0 * M_PI // 0.00001 // 0 //
// random perturbation added to lmk position
#define PL_RANGE  0.10 // 0.30 //


inline double gen_unit_rand() {
    return double( rand() ) / double( RAND_MAX ) * 2.0 - 1.0;
}


// NOTE
// the alignment issue bewtween Eigen and STL needs to be addressed;
// for now we simply remove Eigen members in all vectors

class Camera_g2o {
public:
    Camera_g2o(const size_t id_,
               const double tx_, const double ty_, const double tz_,
               const double qw_, const double qx_, const double qy_, const double qz_,
               const double fx_, const double fy_, const double cx_, const double cy_,
               const int visible_cnt_ = 0) :
        id(id_), tx(tx_), ty(ty_), tz(tz_), qw(qw_), qx(qx_), qy(qy_), qz(qz_),
        fx(fx_), fy(fy_), cx(cx_), cy(cy_), visible_cnt(visible_cnt_) {};

    size_t id;
    //    Eigen::Vector3d twc;
    //    Eigen::Quaterniond qwc;
    double tx, ty, tz;
    double qw, qx, qy, qz;
    double fx, fy, cx, cy;
    //
    int visible_cnt;
};

class Lmk_g2o {
public:
    Lmk_g2o(const size_t id_,
            const double tx_, const double ty_, const double tz_, const int visible_cnt_ = 0) :
        id(id_), tx(tx_), ty(ty_), tz(tz_), visible_cnt(visible_cnt_) {
        tx_c = 0;
        ty_c = 0;
        tz_c = 0;
    };

    size_t id;
    //    Eigen::Vector3d twc;
    double tx, ty, tz;
    double tx_c, ty_c, tz_c;
    //
    int visible_cnt;
};

class Edge_g2o {
public:
    Edge_g2o(const size_t id_c_, const size_t id_l_,
             const double mu_, const double mv_) :
        id_c(id_c_), id_l(id_l_), mu(mu_), mv(mv_) {};
    size_t id_c, id_l;
    double mu, mv;
};


class CSfMSimulator {
public:

    CSfMSimulator(const size_t num_cam_, const size_t num_lmk_, const size_t max_edges_per_lmk_,
                  const double fx_ = 435.0f, const double fy_ = 435.0f,
                  const double cx_ = 320.0f, const double cy_ = 240.0f) {
        //
        num_cam = num_cam_;
        num_lmk = num_lmk_;
        max_edges_per_lmk = max_edges_per_lmk_;
        //
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    };

    ~CSfMSimulator() {
        vec_lmk.clear();
        vec_camera.clear();
        vec_edge.clear();
    };

    bool ProjectLmk2Cam(const Lmk_g2o & lmk_, const Eigen::Vector3d & twc_, const Eigen::Quaterniond & qwc_,
                        double & mu_, double & mv_) {

        //        std::cout << "lmk_.tx = " << lmk_.tx << "; "
        //                  << "lmk_.ty = " << lmk_.ty << "; "
        //                  << "lmk_.tz = " << lmk_.tz << "; " << std::endl;

        //        std::cout << "twc_(0) = " << twc_(0) << "; "
        //                  << "twc_(1) = " << twc_(1) << "; "
        //                  << "twc_(2) = " << twc_(2) << "; " << std::endl;

        //        std::cout << "qwc_.w() = " << qwc_.w() << "; "
        //                  << "qwc_.x() = " << qwc_.x() << "; "
        //                  << "qwc_.y() = " << qwc_.y() << "; "
        //                  << "qwc_.z() = " << qwc_.z() << "; "<< std::endl;

        // 3D in camera coordinates
        Eigen::Matrix3d Rmat = qwc_.toRotationMatrix();
        Eigen::Vector3d Pc = Rmat * Eigen::Vector3d(lmk_.tx, lmk_.ty, lmk_.tz) + twc_;
        double PcX = Pc(0);
        double PcY = Pc(1);
        double PcZ = Pc(2);

        //        std::cout << "PcX = " << PcX << "; "
        //                  << "PcY = " << PcY << "; "
        //                  << "PcZ = " << PcZ << "; " << std::endl;

        // Check positive depth
        if (PcZ < 0.0 || PcZ > MAX_VISIBLE_DIST)
            return false ;

        // Project in image and check it is not outside
        const double invz = 1.0 / PcZ;
        mu_ = fx * PcX * invz + cx;
        mv_ = fy * PcY * invz + cy;

        //        std::cout << "mu_ = " << mu_ << "; "
        //                  << "mv_ = " << mv_ << "; " << std::endl;

        if (mu_ < 0 || mu_ > 2*cx)
            return false ;
        if (mv_ < 0 || mv_ > 2*cy)
            return false ;

        return true;
    }

    void createSfM(size_t scene_type, size_t rnd_seed) {
        //
        vec_lmk.clear();
        vec_camera.clear();
        vec_edge.clear();

        srand(rnd_seed);

        // spawn random lmks
        for (size_t i=0; i<num_lmk; ++i) {
            double radius, theta;
            switch(scene_type) {
            case 0:
            case 1:
                // pure random spawn OR random lmk + rand contineous motion
                vec_lmk.push_back(Lmk_g2o(i + num_cam,
                                          gen_unit_rand() * WORLD_RANGE,
                                          gen_unit_rand() * WORLD_RANGE,
                                          gen_unit_rand() * WORLD_RANGE, 0));
                break;
            case 2:
                // circular motion with lmk in the center
                radius = fabs(gen_unit_rand()) * WORLD_RANGE * 0.5;
                theta = fabs(gen_unit_rand()) * 2 * M_PI;
                //                radius = WORLD_RANGE * 0.5;
                //                theta = double(i) / double(num_lmk) * 2 * M_PI;
                vec_lmk.push_back(Lmk_g2o(i + num_cam,
                                          cos(theta) * radius,
                                          sin(theta) * radius,
                                          gen_unit_rand() * WORLD_RANGE * 0.1, 0));
                break;
            case 3:
                // circular motion with lmk at the outer side
                radius = fabs(gen_unit_rand()) * WORLD_RANGE * 0.5 + WORLD_RANGE * 1.5;
                theta = fabs(gen_unit_rand()) * 2 * M_PI;
                vec_lmk.push_back(Lmk_g2o(i + num_cam,
                                          cos(theta) * radius,
                                          sin(theta) * radius,
                                          gen_unit_rand() * WORLD_RANGE * 0.1, 0));
                break;
            default:
                //
                break;
            }
            //            std::cout << "Spawn lmk " << i + num_cam << std::endl;
        }

        // spawn random cams
        size_t i=0;
        Eigen::Vector3d twc_prev;
        Eigen::Vector3d ewc_prev;
        while (i<num_cam){
            //
            Eigen::Vector3d twc, ewc;
            Eigen::Quaterniond qwc;
            switch(scene_type) {
            double radius, theta;
            case 0:
                // pure random camera motion
                twc = Eigen::Vector3d(
                            gen_unit_rand() * WORLD_RANGE,
                            gen_unit_rand() * WORLD_RANGE,
                            gen_unit_rand() * WORLD_RANGE
                            );
                ewc = Eigen::Vector3d(
                            fabs(gen_unit_rand()) * 2 * M_PI,
                            fabs(gen_unit_rand()) * 2 * M_PI,
                            fabs(gen_unit_rand()) * 2 * M_PI
                            );
                break;
            case 1:
                // random contineous motion
                if (i == 0) {
                    twc = Eigen::Vector3d(
                                gen_unit_rand() * WORLD_RANGE,
                                gen_unit_rand() * WORLD_RANGE,
                                gen_unit_rand() * WORLD_RANGE
                                );
                    ewc = Eigen::Vector3d(
                                fabs(gen_unit_rand()) * 2 * M_PI,
                                fabs(gen_unit_rand()) * 2 * M_PI,
                                fabs(gen_unit_rand()) * 2 * M_PI
                                );
                }
                else {
                    twc = twc_prev + Eigen::Vector3d(
                                gen_unit_rand() * T_STEP,
                                gen_unit_rand() * T_STEP,
                                gen_unit_rand() * T_STEP
                                );
                    ewc = ewc_prev + Eigen::Vector3d(
                                gen_unit_rand() * E_STEP,
                                gen_unit_rand() * E_STEP,
                                gen_unit_rand() * E_STEP
                                );
                }
                break;
            case 2:
                // circular motion
                radius = 5; // WORLD_RANGE;
                theta = double(i) / double(num_cam) * 2 * M_PI;
            {
                Eigen::Matrix3d rotComb;
                //                rotComb = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()) *
                //                        Eigen::AngleAxisd(M_PI * 3.0 / 2.0, Eigen::Vector3d::UnitY());
                //                rotComb = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX());
                //                rotComb = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
                //                        Eigen::AngleAxisd(M_PI * 3.0 / 2.0, Eigen::Vector3d::UnitY());
                rotComb = Eigen::AngleAxisd(M_PI * 3.0 / 2.0, Eigen::Vector3d::UnitY());
                //                std::cout << theta << std::endl;
                //                std::cout << rotComb << std::endl;
                Eigen::AngleAxisd rotAA;
                rotAA.fromRotationMatrix(rotComb);
                ewc = Eigen::Vector3d(
                            rotAA.axis()
                            ) * rotAA.angle();
                //                twc = rotComb.inverse() * Eigen::Vector3d(
                //                            cos(theta) * radius,
                //                            sin(theta) * radius,
                //                            0);
                twc = Eigen::Vector3d(
                            cos(theta) * radius,
                            sin(theta) * radius,
                            0);
            }
                break;
            case 3:
                // circular motion
                radius = WORLD_RANGE * 0.9;
                theta = double(i) / double(num_cam) * 2 * M_PI;
            {
                Eigen::Matrix3d rotComb;
                rotComb = Eigen::AngleAxisd(M_PI * 3.0 / 2.0, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd rotAA;
                rotAA.fromRotationMatrix(rotComb);
                ewc = Eigen::Vector3d(
                            rotAA.axis()
                            ) * rotAA.angle();
                twc = Eigen::Vector3d(
                            cos(theta) * radius,
                            sin(theta) * radius,
                            0);
            }
                break;
            default:
                break;
            }
            C3DJacobians::AxisAngle_to_Quat(ewc, qwc);

            // spawn edges
            //            std::cout << "Establish edge ";
            int visible_num = 0;
            double u, v;
            for (size_t j=0; j<num_lmk; ++j) {
                if (ProjectLmk2Cam(vec_lmk[j], twc, qwc, u, v) && visible_num < 150)
                {
                    vec_edge.push_back(Edge_g2o(i, vec_lmk[j].id, u, v));
                    visible_num ++;
                    vec_lmk[j].visible_cnt ++;
                    //                std::cout << visible_num << ", ";
                }
            }
            //            std::cout << std::endl;
            //            std::cout << visible_num << std::endl;
            // if edge number is less than 3, skip
            if (visible_num < 3) {
                while (visible_num > 0) {
                    visible_num --;
                    vec_lmk[vec_edge.back().id_l - num_cam].visible_cnt --;
                    vec_edge.pop_back();
                }
                //                continue ;
            }
            else {
                twc_prev = twc;
                ewc_prev = ewc;
                vec_camera.push_back(Camera_g2o(i, twc.x(), twc.y(), twc.z(), qwc.w(), qwc.x(), qwc.y(), qwc.z(), fx, fy, cx, cy, visible_num));
                ++i;
                //                std::cout << "Spawn camera " << i << " with " << visible_num << " edges" << std::endl;
            }
        }

        // check lmks, make sure all lmks are observed by at least 2 cameras
        for (size_t i=0; i<num_lmk; ++i) {
            //            std::cout << "Lmk " << vec_lmk[i].id << " is visible in " << vec_lmk[i].visible_cnt << " cameras" << std::endl;
            if (vec_lmk[i].visible_cnt >= 3)
                continue ;
            //
            // remove existing edges for lmk i
            //                size_t Nedge = vec_edge.size();
            for (size_t k=0; k<vec_edge.size(); ++k) {
                if (vec_edge[k].id_l == vec_lmk[i].id) {
                    //
                    vec_edge.erase(vec_edge.begin() + k);
                }
            }
            while (vec_lmk[i].visible_cnt < 3) {
                vec_lmk[i].visible_cnt = 0;
                // respawn random lmk i

                double radius, theta;
                switch(scene_type) {
                case 0:
                case 1:
                    // pure random spawn OR random lmk + rand contineous motion
                    vec_lmk[i].tx = gen_unit_rand() * WORLD_RANGE;
                    vec_lmk[i].ty = gen_unit_rand() * WORLD_RANGE;
                    vec_lmk[i].tz = gen_unit_rand() * WORLD_RANGE;
                    break;
                case 2:
                    // circular motion with lmk in the center
                    radius = fabs(gen_unit_rand()) * WORLD_RANGE * 0.5;
                    theta = fabs(gen_unit_rand()) * 2 * M_PI;
                    vec_lmk[i].tx = cos(theta) * radius;
                    vec_lmk[i].ty = sin(theta) * radius;
                    vec_lmk[i].tz = gen_unit_rand() * WORLD_RANGE * 0.1;
                    break;
                case 3:
                    // circular motion with lmk at the outer side
                    radius = fabs(gen_unit_rand()) * WORLD_RANGE * 0.5 + WORLD_RANGE * 1.5;
                    theta = fabs(gen_unit_rand()) * 2 * M_PI;
                    vec_lmk[i].tx = cos(theta) * radius;
                    vec_lmk[i].ty = sin(theta) * radius;
                    vec_lmk[i].tz = gen_unit_rand() * WORLD_RANGE * 0.1;
                    break;
                default:
                    //
                    break;
                }

                // check the visible camera num
                double u, v;
                for (size_t j=0; j<num_cam; ++j) {
                    if (ProjectLmk2Cam(vec_lmk[i],
                                       Eigen::Vector3d(vec_camera[j].tx, vec_camera[j].ty, vec_camera[j].tz),
                                       Eigen::Quaterniond(vec_camera[j].qw, vec_camera[j].qx, vec_camera[j].qy, vec_camera[j].qz),
                                       u, v)) {
                        vec_lmk[i].visible_cnt ++;
                    }
                }
            }
            // when finding one valid lmk, update edges
            double u, v;
            for (size_t j=0; j<num_cam; ++j) {
                if (ProjectLmk2Cam(vec_lmk[i],
                                   Eigen::Vector3d(vec_camera[j].tx, vec_camera[j].ty, vec_camera[j].tz),
                                   Eigen::Quaterniond(vec_camera[j].qw, vec_camera[j].qx, vec_camera[j].qy, vec_camera[j].qz),
                                   u, v)) {
                    vec_edge.push_back(Edge_g2o(vec_camera[j].id, vec_lmk[i].id, u, v));
                }
            }
        }

        // sparsify edges to reduce covisibility
        for (size_t i=0; i<num_lmk; ++i) {
            if (vec_lmk[i].visible_cnt > max_edges_per_lmk) {
                // randomly reduce visible cameras for vec_lmk[i] to meet max_edges_per_lmk
                std::vector<size_t> vis_edges;
                for (size_t j=0; j<vec_edge.size(); ++j) {
                    if (vec_edge[j].id_l == vec_lmk[i].id) {
                        vis_edges.push_back(j);
                    }
                }
                std::random_shuffle(vis_edges.begin(), vis_edges.end());
                size_t num_edge_rm = vec_lmk[i].visible_cnt - max_edges_per_lmk;
                //                 std::cout << "To remove " << num_edge_rm << " edges for lmk " << vec_lmk[i].id << std::endl;
                vis_edges.erase(vis_edges.begin() + num_edge_rm, vis_edges.end());
                std::sort(vis_edges.begin(), vis_edges.end(), std::greater<size_t>());
                //
                for (size_t j=0; j<vis_edges.size(); ++j) {
                    //                     std::cout << "Removing edge between lmk " << vec_edge[vis_edges[j]].id_l << " and cam " << vec_edge[vis_edges[j]].id_c << std::endl;
                    vec_edge.erase(vec_edge.begin() + vis_edges[j]);
                    vec_lmk[i].visible_cnt --;
                }
                //                 std::cout << "Reduce edges to " << vec_lmk[i].visible_cnt << std::endl;
            }
        }

        assert(vec_edge.size() <= max_edges_per_lmk * num_lmk);
    };

    void writeToG2OFile(const char * fname_g2o, bool do_perturb = false) {
        ofstream fout;
        fout.open(fname_g2o);
        //
        srand(time(NULL));

        for (size_t i=0; i<num_cam; ++i) {
            //            std::cout << "save cam " << i << std::endl;
            if (do_perturb) {
                //                std::cout << "quat before perturb: "
                //                          << vec_camera[i].qw << ", "
                //                          << vec_camera[i].qx << ", "
                //                          << vec_camera[i].qy << ", "
                //                          << vec_camera[i].qz  << std::endl;
                double ntx = gen_unit_rand() * PT_RANGE,
                        nty = gen_unit_rand() * PT_RANGE,
                        ntz = gen_unit_rand() * PT_RANGE;
                double nang = gen_unit_rand() * PA_RANGE;
                //                Eigen::Quaterniond nqwc(sqrt(1 - nqx*nqx  - nqy*nqy  - nqz*nqz),
                //                                        nqx, nqy, nqz);
                Eigen::Vector3d axis;
                C3DJacobians::Quat_to_AxisAngle(
                            Eigen::Quaterniond(vec_camera[i].qw, vec_camera[i].qx, vec_camera[i].qy, vec_camera[i].qz),
                            axis);
                double axis_norm = axis.norm();
                axis.normalize();
                axis = axis * (axis_norm + nang);
                //                axis += Eigen::Vector3d(nax, nay, naz);
                vec_camera[i].tx += ntx;
                vec_camera[i].ty += nty;
                vec_camera[i].tz += ntz;
                //                vec_camera[i].qwc.vec() += nqwc.vec();
                //                vec_camera[i].qwc.w() += nqwc.w();
                //                vec_camera[i].qwc.normalize();
                Eigen::Quaterniond quat;
                C3DJacobians::AxisAngle_to_Quat(axis, quat);
                quat.normalize();
                vec_camera[i].qw = quat.w();
                vec_camera[i].qx = quat.x();
                vec_camera[i].qy = quat.y();
                vec_camera[i].qz = quat.z();
                //                vec_camera[i].qwc.normalize();
            }
            else {
                // do nothing
            }
            //

            Eigen::Quaternion<double> quat(
                        vec_camera[i].qw,
                        vec_camera[i].qx,
                        vec_camera[i].qy,
                        vec_camera[i].qz);
            //            quat.normalize();
            quat = quat.inverse();
            // init translation
            Eigen::Vector3d t_vec(
                        vec_camera[i].tx,
                        vec_camera[i].ty,
                        vec_camera[i].tz);
            //rotate
            Eigen::Vector3d c = quat * (-t_vec);

            fout << "VERTEX_CAM "
                 << std::setprecision(0)
                 << vec_camera[i].id << " "
                 << std::setprecision(5)
                    //                 << vec_camera[i].tx << " " << vec_camera[i].ty << " " << vec_camera[i].tz << " "
                    //                 << vec_camera[i].qx << " " << vec_camera[i].qy << " " << vec_camera[i].qz << " " << vec_camera[i].qw << " "
                 << c.x() << " " << c.y() << " " << c.z() << " "
                 << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
                 << vec_camera[i].fx << " " << vec_camera[i].fy << " " << vec_camera[i].cx << " " << vec_camera[i].cy << " 0\n";
        }
        //
        for (size_t i=0; i<num_lmk; ++i) {
            //            std::cout << "save lmk " << i << std::endl;
            if (do_perturb) {
                //                std::cout << "before perturb: " << vec_lmk[i].twc.x() << ", " << vec_lmk[i].twc.y() << ", " << vec_lmk[i].twc.z() << std::endl;
                vec_lmk[i].tx += gen_unit_rand() * PL_RANGE;
                vec_lmk[i].ty += gen_unit_rand() * PL_RANGE;
                vec_lmk[i].tz += gen_unit_rand() * PL_RANGE;
                //                std::cout << "added perturb: " << ntwc.x() << ", " << ntwc.y() << ", " << ntwc.z() << std::endl;
                //                std::cout << "after perturb: " << vec_lmk[i].twc.x() << ", " << vec_lmk[i].twc.y() << ", " << vec_lmk[i].twc.z() << std::endl;
            }
            else {
                // do nothing
            }
            //
            fout << "VERTEX_XYZ "
                 << std::setprecision(0)
                 << vec_lmk[i].id << " "
                 << std::setprecision(5)
                 << vec_lmk[i].tx << " " << vec_lmk[i].ty << " " << vec_lmk[i].tz << "\n";
        }
        //
        size_t N = vec_edge.size();
        for (size_t i=0; i<N; ++i) {
            fout << "EDGE_PROJECT_P2MC "
                 << std::setprecision(0)
                 << vec_edge[i].id_l << " " << vec_edge[i].id_c << " "
                 << std::setprecision(5)
                 << vec_edge[i].mu << " " << vec_edge[i].mv << " 1 0 1\n";
        }

        fout.close();
    };

    void createRefPCD(size_t ref_cam_idx) {
        _ASSERTE(!vec_lmk.empty() && num_lmk == vec_lmk.size());
        _ASSERTE(ref_cam_idx < vec_camera.size());
        //
        Eigen::Vector3d twc(vec_camera[ref_cam_idx].tx,
                            vec_camera[ref_cam_idx].ty,
                            vec_camera[ref_cam_idx].tz);
        Eigen::Quaterniond qwc(vec_camera[ref_cam_idx].qw,
                               vec_camera[ref_cam_idx].qx,
                               vec_camera[ref_cam_idx].qy,
                               vec_camera[ref_cam_idx].qz);
        Eigen::Matrix3d Rwc = qwc.toRotationMatrix();
        for (size_t i=0; i<num_lmk; ++i) {
            Eigen::Vector3d Pc = Rwc * Eigen::Vector3d(vec_lmk[i].tx, vec_lmk[i].ty, vec_lmk[i].tz) + twc;
            vec_lmk[i].tx_c = Pc.x();
            vec_lmk[i].ty_c = Pc.y();
            vec_lmk[i].tz_c = Pc.z();
        }
    }

    double getRMSE(CBAOptimizer * optim_data_) {
        //
        _ASSERTE(optim_data_ != NULL);

        bool gotFirstCam = false;
        Eigen::Vector3d twc;
        Eigen::Quaterniond qwc;
        Eigen::Matrix3d Rwc;
        size_t vertex_sz_ = optim_data_->n_Vertex_Num();
        for (size_t i=0; i<vertex_sz_; ++i) {
            //            std::cout << int(optim_data_->r_Vertex_State(i).rows()) << std::endl;
            if (int(optim_data_->r_Vertex_State(i).rows()) == 6 && gotFirstCam == false) {
                twc.x() = optim_data_->r_Vertex_State(i)[0];
                twc.y() = optim_data_->r_Vertex_State(i)[1];
                twc.z() = optim_data_->r_Vertex_State(i)[2];
                Eigen::Vector3d axis(optim_data_->r_Vertex_State(i)[3],
                        optim_data_->r_Vertex_State(i)[4],
                        optim_data_->r_Vertex_State(i)[5]);
                C3DJacobians::AxisAngle_to_Quat(axis, qwc);
                Rwc = qwc.toRotationMatrix();
                //
                gotFirstCam = true;
                break ;
            }
        }

        // failed to find reference camera, terminate
        if (!gotFirstCam) {
            std::cout << "Failed to find a reference camera" << std::endl;
            return -1;
        }

        // search for NN in the reference PCD
        std::vector<double> dist_arr;
        for (size_t i=0; i<vertex_sz_; ++i) {
            if (int(optim_data_->r_Vertex_State(i).rows()) == 3) {
                // only take vertexxyz (lmk)
                //                std::cout << "retrive opt. lmk " << i <<  " (in world frame): "
                //                          << optim_data_->r_Vertex_State(i).x() << ", "
                //                          << optim_data_->r_Vertex_State(i).y() << ", "
                //                          << optim_data_->r_Vertex_State(i).z() << ", "
                //                          << std::endl;
                Eigen::Vector3d tp(optim_data_->r_Vertex_State(i).x(),
                                   optim_data_->r_Vertex_State(i).y(),
                                   optim_data_->r_Vertex_State(i).z());
                //
                Eigen::Vector3d Pc = Rwc * tp + twc;
                //                std::cout << "retrive opt. lmk " << i << " (in cam frame): "
                //                          << Pc.x() << ", "
                //                          << Pc.y() << ", "
                //                          << Pc.z() << ", "
                //                          << std::endl;
                double dist_min = DBL_MAX;
                for (size_t j=0; j<num_lmk; ++j) {
                    Eigen::Vector3d Pr(vec_lmk[j].tx_c,
                                       vec_lmk[j].ty_c,
                                       vec_lmk[j].tz_c);
                    double dist_tmp = double( (Pr - Pc).norm() );
                    if (dist_tmp < dist_min) {
                        dist_min = dist_tmp;
                    }
                }
                dist_arr.push_back(dist_min);
            }
        }
        if (dist_arr.empty())
            return -1;

        //
        //        return sqrt( std::inner_product( dist_arr.begin(), dist_arr.end(), dist_arr.begin(), 0 ) /
        //                     static_cast<double>( dist_arr.size() ) );
        return rmseFromVector(dist_arr);
    }

private:

    double rmseFromVector(const std::vector<double> & data_arr)
    {
        double square = 0;
        double rms = 0.0;

        // Calculate square.
        size_t n = data_arr.size();
        for (size_t i = 0; i < n; i++) {
            square += pow(data_arr[i], 2);
        }

        // Calculate Root.
        rms = sqrt(square / static_cast<double>(n));

        return rms;
    }

    size_t num_cam, num_lmk, max_edges_per_lmk;
    double fx, fy, cx, cy;

    // Grount truth states
    std::vector<Camera_g2o> vec_camera;
    std::vector<Lmk_g2o>    vec_lmk;
    std::vector<Edge_g2o>   vec_edge;
};
