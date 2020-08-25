/*
                                +-----------------------------------+
                                |                                   |
                                | ***  Simple BA iface example  *** |
                                |                                   |
                                |  Copyright  (c) -tHE SWINe- 2015  |
                                |                                   |
                                |             Main.cpp              |
                                |                                   |
                                +-----------------------------------+
*/

/**
 *	@file src/ba_interface_example/Main.cpp
 *	@brief contains the main() function of the simple example BA program
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include <stdio.h> // printf
#include <fstream>
#include <string>
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include "good_graph_testbed/BAOptimizer.h" // BA types
// note that nothing else from SLAM++ needs to be included

#include "slam/Parser.h" // want to be able to parse .graph files
#include "slam_app/ParsePrimitives.h" // reuse SLAM++ standard parse primitives
// we need this to parse .graph files

int n_dummy_param = 0; // required by the DL solver, otherwise causes a link error

// perform 100-repeat test of lazier greedy vs. greedy baseline
//#define EVAL_LAZIER_GREEDY

// compare good graph with other subset selection methods, using real SfM benchmark
#define EVAL_SUBSET_METHODS_REAL

// compare good graph with other subset selection methods, using random simulated SfM problem
//#define EVAL_SUBSET_METHOD_SIMU

#ifdef EVAL_SUBSET_METHOD_SIMU
// when uncommented, sweep the choice of lazier greedy factor
// otherwise, sweep the choice of covisbile candidate pool ratio
//#define SWEEP_LAZIER_FACTOR

#include <string>
#include "good_graph_testbed/SfMSimulator.hpp"
#endif

#define SIMU_ROUND      1 // 50 // 10 //

#define OPT_ITER_NUM    20 // 100 //

/**
 *	@brief a simple parse loop for BA systems
 */
class CMyParseLoop {
protected:
    CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */

public:
    /**
     *	@brief default constructor
     *	@param[in] r_optimizer is reference to the optimizer to be filled with edges and vertices
     */
    CMyParseLoop(CBAOptimizer &r_optimizer)
        :m_r_optimizer(r_optimizer)
    {}

    /**
     *	@brief appends the system with a general edge of unknown type
     *	@param[in] r_t_edge is the measurement to be appended
     */
#ifdef ENABLE_STEREO_SLAMPP
    // hack function to get around the compile error
    void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
    {
        //        m_r_optimizer.Add_P2SC3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
        //                                    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
    }
#elif defined ENABLE_QUATERNION
    void AppendSystem(const CParserBase::TEdgeP2C3D_Quat &r_t_edge) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_P2C3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
                                    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
    }
#else
    void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_P2C3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
                                    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
        //
        map_pose_2_lmk[r_t_edge.m_n_node_1].push_back(r_t_edge.m_n_node_0);
        map_lmk_2_pose[r_t_edge.m_n_node_0].push_back(r_t_edge.m_n_node_1);
    }
#endif


    /**
     *	@brief initializes a camera vertex
     *	@param[in] r_t_vertex is the vertex to be initialized
     */
#ifdef ENABLE_STEREO_SLAMPP
    // hack function to get around the compile error
    void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
    {
        //        m_r_optimizer.Add_SCamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        //
        //        poses_vtx_.push_back(r_t_vertex.m_n_id);
    }
#elif defined ENABLE_QUATERNION
    void InitializeVertex(const CParserBase::TVertexCam3D_Quat &r_t_vertex) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_CamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        //
        poses_vtx_.push_back(r_t_vertex.m_n_id);
    }
#else
    void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_CamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        //
        poses_vtx_.push_back(r_t_vertex.m_n_id);
    }
#endif


    /**
     *	@brief initializes a structure point vertex
     *	@param[in] r_t_vertex is the vertex to be initialized
     */
    void InitializeVertex(const CParserBase::TVertexXYZ &r_t_vertex) // throw(std::bad_alloc)
    {
        m_r_optimizer.Add_XYZVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
        lmks_vtx_.push_back(r_t_vertex.m_n_id);
    }

    //
    std::vector<size_t> poses_vtx_;
    std::vector<size_t> reserv_pos_;
    std::vector<size_t> lmks_vtx_;
    //
    std::map<size_t, std::vector<size_t>> map_pose_2_lmk;
    std::map<size_t, std::vector<size_t>> map_lmk_2_pose;
};


//struct camStruct {
//    int id;
//    // c[0], c[1], c[2], axis(0), axis(1), axis(2), pKF->fx, pKF->fy, pKF->cx, pKF->cy, 0;
//    Eigen::Matrix<double, 11, 1> pose;
//    std::vector<int> lmk_ids;
//};

//struct lmkStruct {
//    int id;
//    //    double x, y, z;
//    Eigen::Vector3d pose;
//};

void splitString(const std::string & s,
                 const std::string & delimiter,
                 std::vector<std::string> &res) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    res.clear();

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
}

void BuildSubGraph(const char * p_s_input_file, const std::vector<size_t> & cam_subset,
                   CBAOptimizer & optimizer) {
    //
    size_t nidx = 0;
    // old id -> new id
    std::map<size_t, size_t> map_cam_subset, map_lmk_subset;
    std::map<size_t, int> map_lmk_tmp;
    for (const auto & ele : cam_subset)
    {
        map_cam_subset.insert({ele, nidx});
        nidx ++;
    }

    std::ifstream fin;
    // Load all camera ids for subgraph
    fin.open(p_s_input_file);
    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line)) {
            //            std::cout << line << std::endl;
            //
            std::vector<std::string> words;
            splitString(line, std::string(" "), words);
            if (words.size() > 0) {
                if (words[0] == "VERTEX_CAM") {
                    assert(words.size() == 14);
                    size_t id = std::stoi(words[1]);
                    auto iter = map_cam_subset.find(id);
                    if (iter == map_cam_subset.end()) {
                        // std::cout << "cannot find the new cam index " << id << " in the hash!" << std::endl;
                        continue ;
                    }
                    // init rotation
                    Eigen::Quaternion<double> quat(
                                std::stod(words[8]),
                            std::stod(words[5]),
                            std::stod(words[6]),
                            std::stod(words[7]));
                    quat.normalize();
                    quat = quat.inverse();
                    //Eigen::Matrix3d Q = quat.toRotationMatrix();
                    //Q = Q.inverse().eval();
                    //Q = Q.householderQr().householderQ();
                    // init translation
                    Eigen::Vector3d t_vec(
                                std::stod(words[2]),
                            std::stod(words[3]),
                            std::stod(words[4]));
                    //rotate
                    Eigen::Vector3d c = quat * (-t_vec);
                    //Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
                    Eigen::Vector3d axis;
                    C3DJacobians::Quat_to_AxisAngle(quat, axis);
                    // assemble into eigen matrix
                    Eigen::Matrix<double, 11, 1> pose_camera_;
                    pose_camera_ << c[0], c[1], c[2], axis(0), axis(1), axis(2), std::stod(words[9]), std::stod(words[10]), std::stod(words[11]), std::stod(words[12]), std::stod(words[13]);
                    // insert to the flat system
                    //                    std::cout << "create cam with id " << iter->second << std::endl;
                    optimizer.Add_CamVertex(iter->second, pose_camera_);
                }
                else if (words[0] == "VERTEX_XYZ") {
                    assert(words.size() == 5);
                    // do nothing at this point
                }
                else if (words[0] == "EDGE_PROJECT_P2MC") {
                    assert(words.size() == 8);
                    // do nothing at this point
                }
                else if (words[0] == "EDGE_PROJECT_P2SC") {
                    assert(words.size() == 12);
                    // do nothing at this point
                }
                else {
                    std::cout << "error: no defined graph components " << words[0] << " from file" << std::endl;
                }
            }
        }
        fin.close();
    }

    // Load all edges for subgraph
    fin.open(p_s_input_file);
    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line)) {
            std::vector<std::string> words;
            splitString(line, std::string(" "), words);
            if (words.size() > 0) {
                if (words[0] == "VERTEX_CAM") {
                    assert(words.size() == 14);
                    // do nothing at this point
                }
                else if (words[0] == "VERTEX_XYZ") {
                    assert(words.size() == 5);
                    // do nothing at this point
                }
                else if (words[0] == "EDGE_PROJECT_P2MC") {
                    assert(words.size() == 8);

                    size_t lmk_id = std::stoi(words[1]), kf_id = std::stoi(words[2]);
                    if (map_cam_subset.find(kf_id) == map_cam_subset.end())
                        continue ;
                    auto it = map_lmk_tmp.find(lmk_id);
                    if (it != map_lmk_tmp.end()) {
                        // already in the map; increase visible count
                        // std::cout << "update lmk pair " << lmk_id << ", " << vis_cnt << std::endl;
                        // map_lmk_tmp.insert({lmk_id, vis_cnt});
                        it->second ++;
                        continue ;
                    }
                    // std::cout << "insert lmk pair " << lmk_id << ", " << 1 << std::endl;
                    //                    map_lmk_subset.insert({lmk_id, nidx});
                    //                    nidx ++;
                    map_lmk_tmp.insert({lmk_id, 1});
                }
                else if (words[0] == "EDGE_PROJECT_P2SC") {
                    assert(words.size() == 12);

                    size_t lmk_id = std::stoi(words[1]), kf_id = std::stoi(words[2]);
                    if (map_cam_subset.find(kf_id) == map_cam_subset.end())
                        continue ;
                    auto it = map_lmk_tmp.find(lmk_id);
                    if (it != map_lmk_tmp.end()) {
                        // already in the map; increase visible count
                        // std::cout << "update lmk pair " << lmk_id << ", " << vis_cnt << std::endl;
                        // map_lmk_tmp.insert({lmk_id, vis_cnt});
                        it->second ++;
                        continue ;
                    }
                    map_lmk_tmp.insert({lmk_id, 1});
                }
                else {
                    std::cout << "error: no defined graph components " << words[0] << " from file" << std::endl;
                }
            }
        }
        fin.close();
    }

    std::cout << nidx << std::endl;


    // Fill in lmk idx with at least 2 views
    // reset the new id for map points according to the order of key string (original id)
    for (auto it=map_lmk_tmp.begin(); it!=map_lmk_tmp.end(); ++it)
        // for (size_t i=0; i<map_lmk_tmp.size(); ++i)
    {
        //
        // due to the natrual of Schur marginalization, all visible lmks in the
        // selected subsystem contributes; more importantly, the lmks that are
        // visible to unselected cameras are counted here as well (as linearized
        // constraint)!
        //
        if (it->second <= 1) {
            //            map_lmk_subset.insert({it->first, -1});
            //            it->second = -1;
        }
        else {
            map_lmk_subset.insert({it->first, nidx});
            //            it->second = nidx;
            nidx ++;
        }
        //        std::cout << it->first << " -> " << it->second << std::endl;
    }
    std::cout << nidx << std::endl;

    std::cout << "Done with 1st scan with total idx num " << nidx << "; all tables are constructed!" << std::endl;

    // Scan file again to fill in selected lmks
    fin.open(p_s_input_file);
    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line)) {
            //                        std::cout << line << std::endl;
            //
            std::vector<std::string> words;
            splitString(line, std::string(" "), words);
            if (words.size() > 0) {
                if (words[0] == "VERTEX_CAM") {
                    // do nothing since already inserted
                }
                else if (words[0] == "VERTEX_XYZ") {
                    assert(words.size() == 5);
                    size_t lmk_id = std::stoi(words[1]);
                    auto iter = map_lmk_subset.find(lmk_id);
                    if (iter == map_lmk_subset.end())// || iter->second == -1)
                    {
                        //                        std::cout << "cannot find lmk with id " << lmk_id << std::endl;
                        continue ;
                    }
                    Eigen::Vector3d pose_lmk_;
                    pose_lmk_ << std::stod(words[2]), std::stod(words[3]), std::stod(words[4]);
                    //
                    //                    std::cout << "create lmk with new id " << iter->second << std::endl;
                    optimizer.Add_XYZVertex(iter->second, pose_lmk_);
                }
                else if (words[0] == "EDGE_PROJECT_P2MC") {
                    //                    assert(words.size() == 8);
                    //                    int lmk_id = std::stoi(words[1]);
                    //                    auto iter_1 = map_lmk_subset.find(lmk_id);
                    //                    if (iter_1 == map_lmk_subset.end())
                    //                        continue ;
                    //                    int kf_id = std::stoi(words[2]);
                    //                    auto iter_2 = map_cam_subset.find(kf_id);
                    //                    if (iter_2 == map_cam_subset.end())
                    //                        continue ;
                    //                    Eigen::Vector2d v_observation;
                    //                    v_observation << std::stod(words[3]), std::stod(words[4]);
                    //                    Eigen::Matrix2d v_infomat;
                    //                    v_infomat.setIdentity();
                    //                    v_infomat(0, 0) = std::stod(words[5]);
                    //                    v_infomat(0, 1) = std::stod(words[6]);
                    //                    v_infomat(1, 1) = std::stod(words[7]);

                    //                    optimizer.Add_P2C3DEdge(static_cast<size_t>(iter_1->second), static_cast<size_t>(iter_2->second),
                    //                                            v_observation, v_infomat);
                }
                else if (words[0] == "EDGE_PROJECT_P2SC") {
                    //                    assert(words.size() == 12);
                    //                    int lmk_id = std::stoi(words[1]);
                    //                    auto iter_1 = map_lmk_subset.find(lmk_id);
                    //                    if (iter_1 == map_lmk_subset.end())
                    //                        continue ;
                    //                    int kf_id = std::stoi(words[2]);
                    //                    auto iter_2 = map_cam_subset.find(kf_id);
                    //                    if (iter_2 == map_cam_subset.end())
                    //                        continue ;
                    //                    Eigen::Vector2d v_observation;
                    //                    v_observation << std::stod(words[3]), std::stod(words[4]);
                    //                    Eigen::Matrix2d v_infomat;
                    //                    v_infomat.setIdentity();
                    //                    v_infomat(0, 0) = std::stod(words[6]);
                    //                    v_infomat(0, 1) = std::stod(words[7]);
                    //                    v_infomat(1, 1) = std::stod(words[11]);

                    //                    optimizer.Add_P2C3DEdge(static_cast<size_t>(iter_1->second), static_cast<size_t>(iter_2->second),
                    //                                            v_observation, v_infomat);
                }
                else {
                    //                    fprintf(stderr, "error: no defined graph components from file\n");
                    std::cout << "error: no defined graph components " << words[0] << " from file" << std::endl;
                }
            }
        }
        fin.close();

    }
    std::cout << "Done with 2nd scan with all vertex filled in!" << std::endl;

    // Scan file again to fill in subgraph edges
    fin.open(p_s_input_file);
    if (fin.is_open()) {
        std::string line;
        while (std::getline(fin, line)) {
            //                        std::cout << line << std::endl;
            //
            std::vector<std::string> words;
            splitString(line, std::string(" "), words);
            if (words.size() > 0) {
                if (words[0] == "VERTEX_CAM") {
                    // do nothing since already inserted
                }
                else if (words[0] == "VERTEX_XYZ") {
                    // do nothing since already inserted
                }
                else if (words[0] == "EDGE_PROJECT_P2MC") {
                    assert(words.size() == 8);
                    size_t lmk_id = std::stoi(words[1]);
                    auto iter_1 = map_lmk_subset.find(lmk_id);
                    if (iter_1 == map_lmk_subset.end())
                        continue ;
                    size_t kf_id = std::stoi(words[2]);
                    auto iter_2 = map_cam_subset.find(kf_id);
                    if (iter_2 == map_cam_subset.end())
                        continue ;
                    Eigen::Vector2d v_observation;
                    v_observation << std::stod(words[3]), std::stod(words[4]);
                    Eigen::Matrix2d v_infomat;
                    v_infomat.setIdentity();
                    v_infomat(0, 0) = std::stod(words[5]);
                    v_infomat(0, 1) = std::stod(words[6]);
                    v_infomat(1, 1) = std::stod(words[7]);

                    //                    std::cout << "create edge with between cam " << iter_2->second << " and " << iter_1->second << std::endl;
                    optimizer.Add_P2C3DEdge(iter_1->second, iter_2->second,
                                            v_observation, v_infomat);
                }
                else if (words[0] == "EDGE_PROJECT_P2SC") {
                    assert(words.size() == 12);
                    size_t lmk_id = std::stoi(words[1]);
                    auto iter_1 = map_lmk_subset.find(lmk_id);
                    if (iter_1 == map_lmk_subset.end())
                        continue ;
                    size_t kf_id = std::stoi(words[2]);
                    auto iter_2 = map_cam_subset.find(kf_id);
                    if (iter_2 == map_cam_subset.end())
                        continue ;
                    Eigen::Vector2d v_observation;
                    v_observation << std::stod(words[3]), std::stod(words[4]);
                    Eigen::Matrix2d v_infomat;
                    v_infomat.setIdentity();
                    v_infomat(0, 0) = std::stod(words[6]);
                    v_infomat(0, 1) = std::stod(words[7]);
                    v_infomat(1, 1) = std::stod(words[11]);

                    //                    std::cout << "create edge with between cam " << iter_2->second << " and " << iter_1->second << std::endl;
                    optimizer.Add_P2C3DEdge(iter_1->second, iter_2->second,
                                            v_observation, v_infomat);
                }
                else {
                    //                    fprintf(stderr, "error: no defined graph components from file\n");
                    std::cout << "error: no defined graph components " << words[0] << " from file" << std::endl;
                }
            }
        }
        fin.close();

    }
    std::cout << "Done with 3rd scan with all edges filled in!" << std::endl;
}


static bool compareWeightStruct(const std::pair<size_t, int> & p, const std::pair<size_t, int> & q) {
    return p.second > q.second;
}


#ifdef EVAL_LAZIER_GREEDY
int testLazierGreedy(const char * p_s_input_file, const bool b_verbose) {

    CBAOptimizer optimizer(b_verbose);
    // create the optimizer object

    //	{
#ifdef ENABLE_QUATERNION
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DQuatParsePrimitive,
                         CEdgeP2C3DQuatParsePrimitive) CMyParsedPrimitives;
#else
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
                         CEdgeP2C3DParsePrimitive) CMyParsedPrimitives;
#endif
    typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
    CMyParseLoop parse_loop(optimizer);
    if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
        return -1;
    }
    //	}
    // initialize the vertices, add edges

    size_t repeat_times = 100;
    size_t N = parse_loop.poses_vtx_.size(), M = N * 0.1; // N-1; //
    printf("total num. of camera poses = %d\n", N);
    printf("target num. of camera poses = %d\n", M);
    //    optimizer.Dump_Graph("graph_orig.graph");

    // set the last camera state un-removable
    parse_loop.reserv_pos_.clear();
    parse_loop.reserv_pos_.push_back(0);
    parse_loop.reserv_pos_.push_back(1);

    // reduce the graph to meet cardinality constr. M
    double logdt_0 = optimizer.Find_Subgraph(M, 0, parse_loop.reserv_pos_);
    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop.reserv_pos_.size(), logdt_0);
    printf("reserv_vtx_: ");
    for (size_t i=0; i<parse_loop.reserv_pos_.size(); ++i)
        printf("%d ", parse_loop.reserv_pos_[i]);
    printf("\n");
    optimizer.Show_Stats();

    double avg_diff_01 = 0, avg_diff_02 = 0;
    std::vector<size_t> diff_set, diff_01, diff_02;
    std::vector<size_t> subgraph_0 = parse_loop.reserv_pos_;
    std::sort(subgraph_0.begin(), subgraph_0.end());
    //    printf("index of selected nodes: ");
    //    for (size_t i=0; i<subgraph_0.size(); ++i)
    //        printf("%d ", subgraph_0[i]);
    //    printf("\n");

    optimizer.resetTime();
    for (size_t i = 0; i < repeat_times; ++i) {
        parse_loop.reserv_pos_.clear();
        parse_loop.reserv_pos_.push_back(0);
        parse_loop.reserv_pos_.push_back(1);
        double logdt_1 = optimizer.Find_Subgraph(M, 1, parse_loop.reserv_pos_);
        //        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop.reserv_pos_.size(), logdt_1);
        //        printf("reserv_vtx_: ");
        //        for (size_t i=0; i<parse_loop.reserv_pos_.size(); ++i)
        //            printf("%d ", parse_loop.reserv_pos_[i]);
        //        printf("\n");
        //        optimizer.Show_Stats();

        std::vector<size_t> subgraph_1 = parse_loop.reserv_pos_;
        std::sort(subgraph_1.begin(), subgraph_1.end());
        diff_set.clear();
        std::set_difference(subgraph_0.begin(), subgraph_0.end(), subgraph_1.begin(), subgraph_1.end(),
                            std::inserter(diff_set, diff_set.begin()));
        diff_01.push_back(diff_set.size());
        avg_diff_01 += diff_set.size();
    }
    avg_diff_01 /= double(repeat_times);
    optimizer.Show_Stats();

    //    optimizer.resetTime();
    //    for (size_t i = 0; i < repeat_times; ++i) {
    //        parse_loop.reserv_pos_.clear();
    //        parse_loop.reserv_pos_.push_back(0);
    //        parse_loop.reserv_pos_.push_back(1);
    //        double logdt_2 = optimizer.Find_Subgraph(M, 2, parse_loop.reserv_pos_);
    //        //        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop.reserv_pos_.size(), logdt_2);
    //        //        printf("reserv_vtx_: ");
    //        //        for (size_t i=0; i<parse_loop.reserv_pos_.size(); ++i)
    //        //            printf("%d ", parse_loop.reserv_pos_[i]);
    //        //        printf("\n");
    //        //        optimizer.Show_Stats();

    //        std::vector<size_t> subgraph_2 = parse_loop.reserv_pos_;
    //        std::sort(subgraph_2.begin(), subgraph_2.end());
    //        diff_set.clear();
    //        std::set_difference(subgraph_0.begin(), subgraph_0.end(), subgraph_2.begin(), subgraph_2.end(),
    //                            std::inserter(diff_set, diff_set.begin()));
    //        diff_02.push_back(diff_set.size());
    //        avg_diff_02 += diff_set.size();
    //    }
    //    avg_diff_02 /= double(repeat_times);
    //    optimizer.Show_Stats();

    printf("average diff: %.03f\n", avg_diff_01);
    for (size_t i=0; i<diff_01.size(); ++i)
        printf("%d ", diff_01[i]);
    printf("\n\n");

    //    printf("average diff: %.03f\n", avg_diff_02);
    //    for (size_t i=0; i<diff_02.size(); ++i)
    //        printf("%d ", diff_02[i]);
    //    printf("\n\n");

    // optimize the system
    //    optimizer.Optimize(20);
    //    optimizer.Show_Stats();

    //    // show the timing, save the results
    //    optimizer.Show_Stats();
    //    optimizer.Dump_Graph("graph_sub.graph");
    //    optimizer.Dump_State("solution.txt");
}
#endif

//
// The first element of input arg parse_loop->reserv_pos_ is the root_vertex
// The output order of parse_loop->reserv_pos_ will be:
// [
// root_vertex
// most-covisible vertex
// 2nd-most-covisible vertex
// ...
// ]
//
void findCovisSubgraph(const size_t N, const size_t M, CMyParseLoop * parse_loop) {
    size_t root_cam = parse_loop->reserv_pos_[0];
    std::vector<std::pair<size_t, int>> weight_vec;
    std::vector<size_t> root_edge = parse_loop->map_pose_2_lmk[root_cam], diff_set;
    std::sort(root_edge.begin(), root_edge.end());
    for (size_t i=0; i<N; ++i) {
        if (parse_loop->poses_vtx_[i] == root_cam)
            continue ;
        std::vector<size_t> tmp_edge = parse_loop->map_pose_2_lmk[parse_loop->poses_vtx_[i]];
        std::sort(tmp_edge.begin(), tmp_edge.end());
        diff_set.clear();
        std::set_difference(root_edge.begin(), root_edge.end(), tmp_edge.begin(), tmp_edge.end(),
                            std::inserter(diff_set, diff_set.begin()));
        //
        int covis_weight = root_edge.size() - diff_set.size();
        weight_vec.push_back({i, covis_weight});
    }
    std::sort(weight_vec.begin(), weight_vec.end(), compareWeightStruct);
    //
    if (M == 0) {
        // special case: find all covisible cameras
        printf("co-vis weight: \n");
        for (size_t i=0; i<N-1 && weight_vec[i].second > 0; ++i) {
            parse_loop->reserv_pos_.push_back(weight_vec[i].first);
            printf("%d ", weight_vec[i].second);
        }
        printf("\n");
        sort(parse_loop->reserv_pos_.begin(), parse_loop->reserv_pos_.end());
    }
    else {
        // find top M covisible cameras
        printf("co-vis weight: \n");
        for (size_t i=0; i<std::min(N-1, M-1); ++i) {
            parse_loop->reserv_pos_.push_back(weight_vec[i].first);
            printf("%d ", weight_vec[i].second);
        }
        printf("\n");
        sort(parse_loop->reserv_pos_.begin(), parse_loop->reserv_pos_.end());
    }
    printf("reserv_vtx_.size() = %d\n", parse_loop->reserv_pos_.size());
    printf("reserv_vtx_: ");
    for (size_t i=0; i<parse_loop->reserv_pos_.size(); ++i)
        printf("%d ", parse_loop->reserv_pos_[i]);
    printf("\n");
}

void findRandSubgraph(const size_t N, const size_t M, CMyParseLoop * parse_loop) {
    std::random_shuffle (parse_loop->poses_vtx_.begin(), parse_loop->poses_vtx_.end());
    for (size_t i=0; i<std::min(N, M+1); ++i) {
        if (parse_loop->poses_vtx_[i] == parse_loop->reserv_pos_[0])
            continue ;
        parse_loop->reserv_pos_.push_back(parse_loop->poses_vtx_[i]);
        if (parse_loop->reserv_pos_.size() == std::min(N, M))
            break ;
    }
    //
    sort(parse_loop->reserv_pos_.begin(), parse_loop->reserv_pos_.end());
    printf("reserv_vtx_.size() = %d\n", parse_loop->reserv_pos_.size());
    printf("reserv_vtx_: ");
    for (size_t i=0; i<parse_loop->reserv_pos_.size(); ++i)
        printf("%d ", parse_loop->reserv_pos_[i]);
    printf("\n");
}

#ifdef EVAL_SUBSET_METHODS_REAL
int testSubgraphBA_realData(const char * p_s_input_file, const bool b_verbose) {

    size_t N, M;
    double logdt;
    //
    bool do_full = false; // true; //
    bool do_good = true; // false; //
    bool do_covi = false; // true; //
    bool do_rand = false; // true; //
    // create the optimizer object
#ifdef ENABLE_QUATERNION
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DQuatParsePrimitive,
                         CEdgeP2C3DQuatParsePrimitive) CMyParsedPrimitives;
#else
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
                         CEdgeP2C3DParsePrimitive) CMyParsedPrimitives;
#endif
    typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;

    // stage 1: full graph optimization
    CBAOptimizer * optimizer_1 = new CBAOptimizer(b_verbose);
    CMyParseLoop parse_loop_1(*optimizer_1);
    if(!CMyParser().Parse(p_s_input_file, parse_loop_1)) {
        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
        return -1;
    }

    // check if isolate map point exist
    //    for (size_t i=0; i<parse_loop_1.lmks_vtx_.size(); ++i) {
    //        size_t lmk_key = parse_loop_1.lmks_vtx_[i];
    //        std::cout << parse_loop_1.map_lmk_2_pose[lmk_key].size() << std::endl;
    //        if (parse_loop_1.map_lmk_2_pose[lmk_key].size() <= 1){
    //            fprintf(stderr, "error: isolate map point exist!\n");
    //            return -1;
    //        }
    //    }

    N = parse_loop_1.poses_vtx_.size();
    //    // cathedral
    //    M = N * 0.10f;
    //venice
    M = N * 0.10f;
    //    M = N * 0.05f;

    if (do_full) {
        printf("\n\n --------------- Start Full Graph Optimization ---------------\n\n");
        //
        optimizer_1->Dump_Graph("graph_fullInit.graph");

        // optimize the system
        optimizer_1->Optimize(OPT_ITER_NUM);
        // show the timing, save the results
        optimizer_1->Show_Stats();
        optimizer_1->Dump_Graph("graph_fullOpt.graph");
        optimizer_1->Dump_State("solution_fullOpt.txt");
        //    optimizer_1->Plot3D("graph_fullOpt.tga");
    }
    delete optimizer_1;

    size_t nidx;
    if (do_good) {
        printf("\n\n --------------- Start Good Graph Optimization ---------------\n\n");
        // stage 2: good subgraph optimization
        CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
        // create the optimizer object
        CMyParseLoop parse_loop_2(*optimizer_2);
        if(!CMyParser().Parse(p_s_input_file, parse_loop_2)) {
            fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
            return -1;
        }

        N = parse_loop_2.poses_vtx_.size();
        printf("total num. of camera poses = %d\n", N);
        printf("target num. of camera poses = %d\n", M);
        // set the last camera state un-removable
        parse_loop_2.reserv_pos_.clear();
        parse_loop_2.reserv_pos_.push_back(0);

        // Phase 1: reduce the size of candidate pool with covisibility
        //        findCovisSubgraph(N, size_t(0), &parse_loop_2);
        //        findCovisSubgraph(N, size_t(M * 9), &parse_loop_2);
        findCovisSubgraph(N, size_t(double(M) * 1.5), &parse_loop_2);
        optimizer_2->Show_Stats();

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_2_pool = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, parse_loop_2.reserv_pos_, *optimizer_2_pool);

        // NOTE
        // reduce the graph to meet cardinality constr. M
        // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
        // assuming the g2o input is sorted in a certain way:
        // ...
        // all camera rows
        // ...
        // a map row
        // all edges linked to the map row
        // ...
        std::vector<size_t> subgraph_in_pool, subgraph_in_orig;
        subgraph_in_pool.push_back(0);
        logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool);

        // extract the sub-selected entries from pool
        subgraph_in_orig.clear();
        for (const auto idx : subgraph_in_pool) {
            subgraph_in_orig.push_back(parse_loop_2.reserv_pos_[idx]);
        }

        sort(subgraph_in_orig.begin(), subgraph_in_orig.end());
        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", subgraph_in_orig.size(), logdt);
        printf("reserv_vtx_: ");
        for (size_t i=0; i<subgraph_in_orig.size(); ++i)
            printf("%d ", subgraph_in_orig[i]);
        printf("\n");
        optimizer_2_pool->Show_Stats();

        /*
            logdt = optimizer_2->Find_Subgraph(M, 1, parse_loop_2.reserv_pos_);
            sort(parse_loop_2.reserv_pos_.begin(), parse_loop_2.reserv_pos_.end());
            printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop_2.reserv_pos_.size(), logdt);
            printf("reserv_vtx_: ");
            for (size_t i=0; i<parse_loop_2.reserv_pos_.size(); ++i)
                printf("%d ", parse_loop_2.reserv_pos_[i]);
            printf("\n");
            optimizer_2->Show_Stats();
*/

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, subgraph_in_orig, *optimizer_2_sub);

        parse_loop_2.reserv_pos_.clear();
        parse_loop_2.reserv_pos_.push_back(0);
        logdt = optimizer_2_sub->Find_Subgraph(M, 1, parse_loop_2.reserv_pos_);
        sort(parse_loop_2.reserv_pos_.begin(), parse_loop_2.reserv_pos_.end());
        printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop_2.reserv_pos_.size(), logdt);

        //
        optimizer_2_sub->Dump_Graph("graph_goodInit.graph");

        // optimize the system
        optimizer_2_sub->Optimize(OPT_ITER_NUM);
        // show the timing, save the results
        optimizer_2_sub->Show_Stats();
        optimizer_2_sub->Dump_Graph("graph_goodOpt.graph");
        optimizer_2_sub->Dump_State("solution_goodOpt.txt");
        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
        delete optimizer_2;
        delete optimizer_2_pool;
        delete optimizer_2_sub;
    }

    if (do_covi) {
        printf("\n\n --------------- Start Co-vis Graph Optimization ---------------\n\n");
        // stage 3: co-vis weight
        CBAOptimizer * optimizer_3 = new CBAOptimizer(b_verbose);
        // create the optimizer object
        CMyParseLoop parse_loop_3(*optimizer_3);
        if(!CMyParser().Parse(p_s_input_file, parse_loop_3)) {
            fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
            return -1;
        }

        N = parse_loop_3.poses_vtx_.size();
        printf("total num. of camera poses = %d\n", N);
        printf("target num. of camera poses = %d\n", M);
        // set the last camera state un-removable
        parse_loop_3.reserv_pos_.clear();
        parse_loop_3.reserv_pos_.push_back(0);

        // reduce the graph to meet cardinality constr. M
        // find the shared portion of edges between parse_loop_3.reserv_pos_[0] and other cameras
        findCovisSubgraph(N, M, &parse_loop_3);
        optimizer_3->Show_Stats();

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_3_sub = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, parse_loop_3.reserv_pos_, *optimizer_3_sub);

        //
        optimizer_3_sub->Dump_Graph("graph_covisInit.graph");

        // optimize the system
        optimizer_3_sub->Optimize(OPT_ITER_NUM);
        // show the timing, save the results
        optimizer_3_sub->Show_Stats();
        optimizer_3_sub->Dump_Graph("graph_covisOpt.graph");
        optimizer_3_sub->Dump_State("solution_covisOpt.txt");
        delete optimizer_3;
        delete optimizer_3_sub;
    }

    if (do_rand) {
        printf("\n\n --------------- Start Random Graph Optimization ---------------\n\n");
        // stage 4: random
        CBAOptimizer * optimizer_4 = new CBAOptimizer(b_verbose);
        // create the optimizer object
        CMyParseLoop parse_loop_4(*optimizer_4);
        if(!CMyParser().Parse(p_s_input_file, parse_loop_4)) {
            fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
            return -1;
        }

        N = parse_loop_4.poses_vtx_.size();
        printf("total num. of camera poses = %d\n", N);
        printf("target num. of camera poses = %d\n", M);
        // set the last camera state un-removable
        parse_loop_4.reserv_pos_.clear();
        parse_loop_4.reserv_pos_.push_back(0);

        // reduce the graph to meet cardinality constr. M
        findRandSubgraph(N, M, &parse_loop_4);
        optimizer_4->Show_Stats();

        // re-config the optimizer with selected subgraph
        CBAOptimizer * optimizer_4_sub = new CBAOptimizer(b_verbose);
        BuildSubGraph(p_s_input_file, parse_loop_4.reserv_pos_, *optimizer_4_sub);

        //
        optimizer_4_sub->Dump_Graph("graph_randInit.graph");

        // optimize the system
        optimizer_4_sub->Optimize(OPT_ITER_NUM);
        // show the timing, save the results
        optimizer_4_sub->Show_Stats();
        optimizer_4_sub->Dump_Graph("graph_randOpt.graph");
        optimizer_4_sub->Dump_State("solution_randOpt.txt");
        delete optimizer_4;
        delete optimizer_4_sub;
    }
}
#endif

#ifdef EVAL_SUBSET_METHOD_SIMU
//
template<typename T>
void saveLog(const char * fname,
             const std::vector<T> & log_full,
             const std::vector<T> & log_good,
             const std::vector<T> & log_good_v2,
             const std::vector<T> & log_good_v3,
             const std::vector<T> & log_good_v4,
             const std::vector<T> & log_good_v5,
             const std::vector<T> & log_good_v6,
             const std::vector<T> & log_covi,
             const std::vector<T> & log_rand,
             size_t & ratio_sz) {
    //
    _ASSERTE(SIMU_ROUND == log_full.size());
    if (ratio_sz == 0)
        ratio_sz = log_good.size() / SIMU_ROUND;
    _ASSERTE(SIMU_ROUND*ratio_sz == log_good.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_good_v2.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_good_v3.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_good_v4.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_good_v5.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_good_v6.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_covi.size());
    _ASSERTE(SIMU_ROUND*ratio_sz == log_rand.size());

    ofstream fout;
    fout.open(fname);
    //
    for (size_t i=0; i<SIMU_ROUND; ++i) {
        fout << log_full[i] << ",";
    }
    fout << endl;
    //[iter*subgraph_ratios.size() + rt]
    for (size_t i=0; i<ratio_sz; ++i) {
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_good[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_good_v2[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_good_v3[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_good_v4[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_good_v5[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_good_v6[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_covi[j*ratio_sz + i] << ",";
        }
        fout << endl;
        //
        for (size_t j=0; j<SIMU_ROUND; ++j) {
            fout << log_rand[j*ratio_sz + i] << ",";
        }
        fout << endl;
    }
    fout.close();
}

int testSubgraphBA_simulation(const char * p_s_input_file, const bool b_verbose) {
    size_t N, M;
    double logdt;
    //
    bool do_full = true; // false; //
    bool do_good_v1 = true; // false; //
    bool do_good_v2 = true;
    bool do_good_v3 = true;
    bool do_good_v4 = true;
    bool do_good_v5 = true;
    bool do_good_v6 = true;
    bool do_covi = true; // false; //
    bool do_rand = true; // false; //
    //
    //    vector<size_t> subgraph_scales{50, 75, 100, 125, 150, 175, 200};
    vector<size_t> subgraph_scales{50};
    //    vector<size_t> subgraph_scales{100};

    vector<double> subgraph_ratios{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    //    vector<double> subgraph_ratios{0.2, 0.2, 0.3, 0.4, 0.5};
    //        vector<double> subgraph_ratios{0.6, 0.7, 0.8, 0.9};

#ifdef SWEEP_LAZIER_FACTOR
    vector<double>  lazier_factor_vec{
        float(std::log(1.0/1e-3)),
                float(std::log(1.0/5e-3)),
                float(std::log(1.0/1e-2)),
                float(std::log(1.0/5e-2)),
                float(std::log(1.0/1e-1)),
                float(std::log(1.0/5e-1))};
#else
    //    vector<double> pool_ratio_vec{
    //        999.0,
    //        2.0,
    //        1.8,
    //        1.6,
    //        1.4,
    //        1.2};
    vector<double> pool_ratio_vec{
        999.0,
        3.0,
        2.5,
        2.0,
        1.5};
#endif

    bool save_simu_data = true;
    size_t ratio_size = subgraph_ratios.size();

    // create the optimizer object
#ifdef ENABLE_QUATERNION
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DQuatParsePrimitive,
                         CEdgeP2C3DQuatParsePrimitive) CMyParsedPrimitives;
#else
    typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
                         CEdgeP2C3DParsePrimitive) CMyParsedPrimitives;
#endif
    typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
    //    double rmse_;
    for (size_t sc=0; sc<subgraph_scales.size(); ++sc) {
        //
        vector<double> rmse_full(SIMU_ROUND, 0),
                rmse_good_v1(SIMU_ROUND*ratio_size, 0),
                rmse_good_v2(SIMU_ROUND*ratio_size, 0),
                rmse_good_v3(SIMU_ROUND*ratio_size, 0),
                rmse_good_v4(SIMU_ROUND*ratio_size, 0),
                rmse_good_v5(SIMU_ROUND*ratio_size, 0),
                rmse_good_v6(SIMU_ROUND*ratio_size, 0),
                rmse_covi(SIMU_ROUND*ratio_size, 0),
                rmse_rand(SIMU_ROUND*ratio_size, 0);
        vector<double> timeCostSub_full(SIMU_ROUND, 0),
                timeCostSub_good_v1(SIMU_ROUND*ratio_size, 0),
                timeCostSub_good_v2(SIMU_ROUND*ratio_size, 0),
                timeCostSub_good_v3(SIMU_ROUND*ratio_size, 0),
                timeCostSub_good_v4(SIMU_ROUND*ratio_size, 0),
                timeCostSub_good_v5(SIMU_ROUND*ratio_size, 0),
                timeCostSub_good_v6(SIMU_ROUND*ratio_size, 0),
                timeCostSub_covi(SIMU_ROUND*ratio_size, 0),
                timeCostSub_rand(SIMU_ROUND*ratio_size, 0);
        vector<double> timeCostOpt_full(SIMU_ROUND, 0),
                timeCostOpt_good_v1(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_good_v2(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_good_v3(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_good_v4(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_good_v5(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_good_v6(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_covi(SIMU_ROUND*ratio_size, 0),
                timeCostOpt_rand(SIMU_ROUND*ratio_size, 0);
        vector<size_t> graphScale_full(SIMU_ROUND, 0),
                graphScale_good_v1(SIMU_ROUND*ratio_size, 0),
                graphScale_good_v2(SIMU_ROUND*ratio_size, 0),
                graphScale_good_v3(SIMU_ROUND*ratio_size, 0),
                graphScale_good_v4(SIMU_ROUND*ratio_size, 0),
                graphScale_good_v5(SIMU_ROUND*ratio_size, 0),
                graphScale_good_v6(SIMU_ROUND*ratio_size, 0),
                graphScale_covi(SIMU_ROUND*ratio_size, 0),
                graphScale_rand(SIMU_ROUND*ratio_size, 0);
        vector<double> logDet_full(SIMU_ROUND, 0),
                logDet_good_raw_v1(SIMU_ROUND*ratio_size, 0),
                logDet_good_raw_v2(SIMU_ROUND*ratio_size, 0),
                logDet_good_raw_v3(SIMU_ROUND*ratio_size, 0),
                logDet_good_raw_v4(SIMU_ROUND*ratio_size, 0),
                logDet_good_raw_v5(SIMU_ROUND*ratio_size, 0),
                logDet_good_raw_v6(SIMU_ROUND*ratio_size, 0),
                logDet_good_ref_v1(SIMU_ROUND*ratio_size, 0),
                logDet_good_ref_v2(SIMU_ROUND*ratio_size, 0),
                logDet_good_ref_v3(SIMU_ROUND*ratio_size, 0),
                logDet_good_ref_v4(SIMU_ROUND*ratio_size, 0),
                logDet_good_ref_v5(SIMU_ROUND*ratio_size, 0),
                logDet_good_ref_v6(SIMU_ROUND*ratio_size, 0),
                logDet_covi(SIMU_ROUND*ratio_size, 0),
                logDet_rand(SIMU_ROUND*ratio_size, 0);

        for (size_t iter=0; iter<SIMU_ROUND; ++iter) {
            //            std::srand( unsigned ( std::time(0) ) + iter );

            // simumate random problem
            //                        CSfMSimulator * simulator_ = new CSfMSimulator(subgraph_scales[sc], 200 * subgraph_scales[sc]);
            //            CSfMSimulator * simulator_ = new CSfMSimulator(subgraph_scales[sc], 120 * subgraph_scales[sc]);
            //            CSfMSimulator * simulator_ = new CSfMSimulator(subgraph_scales[sc], 100 * subgraph_scales[sc],  2);
            CSfMSimulator * simulator_ = new CSfMSimulator(subgraph_scales[sc], 60 * subgraph_scales[sc], 15);
            //            CSfMSimulator * simulator_ = new CSfMSimulator(subgraph_scales[sc], 40 * subgraph_scales[sc], 5);
            //    CSfMSimulator * simulator_ = new CSfMSimulator(2, 100);
            //            simulator_->createSfM(iter + 100);
            simulator_->createSfM( 2, static_cast<size_t>( unsigned ( std::time(NULL) ) + iter ) );
            simulator_->createRefPCD(0);
            if (save_simu_data) {
                std::string fname_GT = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_GT.txt";
                simulator_->writeToG2OFile(fname_GT.c_str(), false);
            }
            std::string fname_data = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_data.txt";
            simulator_->writeToG2OFile(fname_data.c_str(), true);
            //        simulator_->writeToG2OFile(p_s_input_file, true);

            // stage 1: full graph optimization
            CBAOptimizer * optimizer_1 = new CBAOptimizer(b_verbose);
            CMyParseLoop parse_loop_1(*optimizer_1);
            if(!CMyParser().Parse(fname_data.c_str(), parse_loop_1)) {
                fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                return -1;
            }

            N = parse_loop_1.poses_vtx_.size();
            graphScale_full[iter] = optimizer_1->n_Vertex_Num();

            if (do_full) {
                printf("\n\n --------------- Start Full Graph Optimization ---------------\n\n");
                //
                if (save_simu_data) {
                    std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_fullInit.graph";
                    optimizer_1->Dump_Graph(fname_init.c_str());
                }
                std::cout << "init rmse = " << simulator_->getRMSE(optimizer_1) << std::endl;
                logDet_full[iter] = optimizer_1->GetFullSystemLogDet();
                std::cout << "init logDet = " << logDet_full[iter] << std::endl;

                // optimize the system
                optimizer_1->Optimize(OPT_ITER_NUM);
                // show the timing, save the results
                optimizer_1->Show_Stats();

                timeCostOpt_full[iter] = optimizer_1->GetTotalTime();
                if (save_simu_data) {
                    std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_fullOpt.graph";
                    optimizer_1->Dump_Graph(fname_opt.c_str());
                    std::string fname_sol = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_fullOpt.txt";
                    optimizer_1->Dump_State(fname_sol.c_str());
                }
                std::string fname_tga = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_fullOpt.tga";
                optimizer_1->Plot3D(fname_tga.c_str());
                //
                rmse_full[iter] = simulator_->getRMSE(optimizer_1);
                std::cout << "final rmse = " << rmse_full[iter] << std::endl;
            }
            delete optimizer_1;

            for (size_t rt=0; rt<ratio_size; ++rt) {
                M = size_t(double(N) * subgraph_ratios[rt]);

                // Original good graph that takes the full BA graph as selection pool
                if (do_good_v1) {
                    printf("\n\n --------------- Start Good Graph Optimization V1 ---------------\n\n");
                    // stage 2: good subgraph optimization
                    CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_2 = new CMyParseLoop(*optimizer_2);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_2)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_2->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_2->reserv_pos_.clear();
                    parse_loop_2->reserv_pos_.push_back(0);

                    // NOTE
                    // reduce the graph to meet cardinality constr. M
                    // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
                    // assuming the g2o input is sorted in a certain way:
                    // ...
                    // all camera rows
                    // ...
                    // a map row
                    // all edges linked to the map row
                    // ...
#ifdef SWEEP_LAZIER_FACTOR
                    std::cout << "lazier factor = " << lazier_factor_vec[0] << std::endl;
                    logdt = optimizer_2->Find_Subgraph(M, 1, parse_loop_2->reserv_pos_, lazier_factor_vec[0]);
#else
                    logdt = optimizer_2->Find_Subgraph(M, 1, parse_loop_2->reserv_pos_);
#endif
                    sort(parse_loop_2->reserv_pos_.begin(), parse_loop_2->reserv_pos_.end());
                    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", parse_loop_2->reserv_pos_.size(), logdt);
                    printf("reserv_vtx_: ");
                    for (size_t i=0; i<parse_loop_2->reserv_pos_.size(); ++i)
                        printf("%d ", parse_loop_2->reserv_pos_[i]);
                    printf("\n");
                    optimizer_2->Show_Stats();
                    timeCostSub_good_v1[iter*ratio_size + rt] = optimizer_2->GetTotalTime();
                    logDet_good_raw_v1[iter*ratio_size + rt] = logdt;

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_2->reserv_pos_, *optimizer_2_sub);
                    graphScale_good_v1[iter*ratio_size + rt] = optimizer_2_sub->n_Vertex_Num();
                    //                    cout << optimizer_2_sub->m_lmk_count << " vs " << optimizer_2_sub->n_Vertex_Num() << endl;

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodInit.graph";
                        optimizer_2_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_good_ref_v1[iter*ratio_size + rt] = optimizer_2_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_good_ref_v1[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_2_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_2_sub->Show_Stats();

                    timeCostOpt_good_v1[iter*ratio_size + rt] = optimizer_2_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodOpt.graph";
                        optimizer_2_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_2_sub->Dump_State("solution_goodOpt.txt");
                        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
                    }
                    rmse_good_v1[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_2_sub);
                    std::cout << "final rmse = " << rmse_good_v1[iter*ratio_size + rt] << std::endl;

                    delete optimizer_2;
                    delete optimizer_2_sub;
                    delete parse_loop_2;
                }

                // Refined good graph that takes the (oversized) covisible subgraph as selection pool
                if (do_good_v2) {
                    printf("\n\n --------------- Start Good Graph Optimization V2 ---------------\n\n");
                    // stage 2: good subgraph optimization
                    CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_2 = new CMyParseLoop(*optimizer_2);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_2)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_2->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_2->reserv_pos_.clear();
                    parse_loop_2->reserv_pos_.push_back(0);

#ifdef SWEEP_LAZIER_FACTOR
                    // simply copy the pointer
                    CBAOptimizer * optimizer_2_pool = optimizer_2;
#else
                    // Phase 1: reduce the size of candidate pool with covisibility
                    findCovisSubgraph(N, size_t(pool_ratio_vec[1] * float(M)), parse_loop_2);
                    optimizer_2->Show_Stats();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_pool = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_2->reserv_pos_, *optimizer_2_pool);
#endif
                    // NOTE
                    // reduce the graph to meet cardinality constr. M
                    // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
                    // assuming the g2o input is sorted in a certain way:
                    // ...
                    // all camera rows
                    // ...
                    // a map row
                    // all edges linked to the map row
                    // ...
                    std::vector<size_t> subgraph_in_pool, subgraph_in_orig;
                    subgraph_in_pool.push_back(0);
#ifdef SWEEP_LAZIER_FACTOR
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool, lazier_factor_vec[1]);
#else
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool);
#endif

                    // extract the sub-selected entries from pool
                    subgraph_in_orig.clear();
                    for (const auto idx : subgraph_in_pool) {
#ifdef SWEEP_LAZIER_FACTOR
                        subgraph_in_orig.push_back(idx);
#else
                        subgraph_in_orig.push_back(parse_loop_2->reserv_pos_[idx]);
#endif
                    }

                    sort(subgraph_in_orig.begin(), subgraph_in_orig.end());
                    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", subgraph_in_orig.size(), logdt);
                    printf("reserv_vtx_: ");
                    for (size_t i=0; i<subgraph_in_orig.size(); ++i)
                        printf("%d ", subgraph_in_orig[i]);
                    printf("\n");
                    optimizer_2_pool->Show_Stats();
                    timeCostSub_good_v2[iter*ratio_size + rt] = optimizer_2_pool->GetTotalTime();
                    logDet_good_raw_v2[iter*ratio_size + rt] = logdt;

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), subgraph_in_orig, *optimizer_2_sub);
                    graphScale_good_v2[iter*ratio_size + rt] = optimizer_2_sub->n_Vertex_Num();
                    //                    cout << optimizer_2_sub->m_lmk_count << " vs " << optimizer_2_sub->n_Vertex_Num() << endl;

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodInit.graph";
                        optimizer_2_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_good_ref_v2[iter*ratio_size + rt] = optimizer_2_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_good_ref_v2[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_2_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_2_sub->Show_Stats();

                    timeCostOpt_good_v2[iter*ratio_size + rt] = optimizer_2_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodOpt.graph";
                        optimizer_2_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_2_sub->Dump_State("solution_goodOpt.txt");
                        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
                    }
                    rmse_good_v2[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_2_sub);
                    std::cout << "final rmse = " << rmse_good_v2[iter*ratio_size + rt] << std::endl;

                    delete optimizer_2;
#ifndef SWEEP_LAZIER_FACTOR
                    delete optimizer_2_pool;
#endif
                    delete optimizer_2_sub;
                    delete parse_loop_2;
                }

                // Refined good graph that takes the (oversized) covisible subgraph as selection pool
                if (do_good_v3) {
                    printf("\n\n --------------- Start Good Graph Optimization V3 ---------------\n\n");
                    // stage 2: good subgraph optimization
                    CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_2 = new CMyParseLoop(*optimizer_2);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_2)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_2->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_2->reserv_pos_.clear();
                    parse_loop_2->reserv_pos_.push_back(0);

#ifdef SWEEP_LAZIER_FACTOR
                    // simply copy the pointer
                    CBAOptimizer * optimizer_2_pool = optimizer_2;
#else
                    // Phase 1: reduce the size of candidate pool with covisibility
                    findCovisSubgraph(N, size_t(pool_ratio_vec[2] * float(M)), parse_loop_2);
                    optimizer_2->Show_Stats();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_pool = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_2->reserv_pos_, *optimizer_2_pool);
#endif
                    // NOTE
                    // reduce the graph to meet cardinality constr. M
                    // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
                    // assuming the g2o input is sorted in a certain way:
                    // ...
                    // all camera rows
                    // ...
                    // a map row
                    // all edges linked to the map row
                    // ...
                    std::vector<size_t> subgraph_in_pool, subgraph_in_orig;
                    subgraph_in_pool.push_back(0);
#ifdef SWEEP_LAZIER_FACTOR
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool, lazier_factor_vec[2]);
#else
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool);
#endif

                    // extract the sub-selected entries from pool
                    subgraph_in_orig.clear();
                    for (const auto idx : subgraph_in_pool) {
#ifdef SWEEP_LAZIER_FACTOR
                        subgraph_in_orig.push_back(idx);
#else
                        subgraph_in_orig.push_back(parse_loop_2->reserv_pos_[idx]);
#endif
                    }

                    sort(subgraph_in_orig.begin(), subgraph_in_orig.end());
                    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", subgraph_in_orig.size(), logdt);
                    printf("reserv_vtx_: ");
                    for (size_t i=0; i<subgraph_in_orig.size(); ++i)
                        printf("%d ", subgraph_in_orig[i]);
                    printf("\n");
                    optimizer_2_pool->Show_Stats();
                    timeCostSub_good_v3[iter*ratio_size + rt] = optimizer_2_pool->GetTotalTime();
                    logDet_good_raw_v3[iter*ratio_size + rt] = logdt;

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), subgraph_in_orig, *optimizer_2_sub);
                    graphScale_good_v3[iter*ratio_size + rt] = optimizer_2_sub->n_Vertex_Num();
                    //                    cout << optimizer_2_sub->m_lmk_count << " vs " << optimizer_2_sub->n_Vertex_Num() << endl;

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodInit.graph";
                        optimizer_2_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_good_ref_v3[iter*ratio_size + rt] = optimizer_2_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_good_ref_v3[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_2_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_2_sub->Show_Stats();

                    timeCostOpt_good_v2[iter*ratio_size + rt] = optimizer_2_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodOpt.graph";
                        optimizer_2_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_2_sub->Dump_State("solution_goodOpt.txt");
                        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
                    }
                    rmse_good_v3[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_2_sub);
                    std::cout << "final rmse = " << rmse_good_v3[iter*ratio_size + rt] << std::endl;

                    delete optimizer_2;
#ifndef SWEEP_LAZIER_FACTOR
                    delete optimizer_2_pool;
#endif
                    delete optimizer_2_sub;
                    delete parse_loop_2;
                }

                // Refined good graph that takes the (oversized) covisible subgraph as selection pool
                if (do_good_v4) {
                    printf("\n\n --------------- Start Good Graph Optimization V4 ---------------\n\n");
                    // stage 2: good subgraph optimization
                    CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_2 = new CMyParseLoop(*optimizer_2);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_2)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_2->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_2->reserv_pos_.clear();
                    parse_loop_2->reserv_pos_.push_back(0);

#ifdef SWEEP_LAZIER_FACTOR
                    // simply copy the pointer
                    CBAOptimizer * optimizer_2_pool = optimizer_2;
#else
                    // Phase 1: reduce the size of candidate pool with covisibility
                    findCovisSubgraph(N, size_t(pool_ratio_vec[3] * float(M)), parse_loop_2);
                    optimizer_2->Show_Stats();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_pool = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_2->reserv_pos_, *optimizer_2_pool);
#endif
                    // NOTE
                    // reduce the graph to meet cardinality constr. M
                    // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
                    // assuming the g2o input is sorted in a certain way:
                    // ...
                    // all camera rows
                    // ...
                    // a map row
                    // all edges linked to the map row
                    // ...
                    std::vector<size_t> subgraph_in_pool, subgraph_in_orig;
                    subgraph_in_pool.push_back(0);
#ifdef SWEEP_LAZIER_FACTOR
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool, lazier_factor_vec[3]);
#else
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool);
#endif

                    // extract the sub-selected entries from pool
                    subgraph_in_orig.clear();
                    for (const auto idx : subgraph_in_pool) {
#ifdef SWEEP_LAZIER_FACTOR
                        subgraph_in_orig.push_back(idx);
#else
                        subgraph_in_orig.push_back(parse_loop_2->reserv_pos_[idx]);
#endif
                    }

                    sort(subgraph_in_orig.begin(), subgraph_in_orig.end());
                    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", subgraph_in_orig.size(), logdt);
                    printf("reserv_vtx_: ");
                    for (size_t i=0; i<subgraph_in_orig.size(); ++i)
                        printf("%d ", subgraph_in_orig[i]);
                    printf("\n");
                    optimizer_2_pool->Show_Stats();
                    timeCostSub_good_v4[iter*ratio_size + rt] = optimizer_2_pool->GetTotalTime();
                    logDet_good_raw_v4[iter*ratio_size + rt] = logdt;

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), subgraph_in_orig, *optimizer_2_sub);
                    graphScale_good_v4[iter*ratio_size + rt] = optimizer_2_sub->n_Vertex_Num();
                    //                    cout << optimizer_2_sub->m_lmk_count << " vs " << optimizer_2_sub->n_Vertex_Num() << endl;

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodInit.graph";
                        optimizer_2_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_good_ref_v4[iter*ratio_size + rt] = optimizer_2_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_good_ref_v4[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_2_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_2_sub->Show_Stats();

                    timeCostOpt_good_v2[iter*ratio_size + rt] = optimizer_2_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodOpt.graph";
                        optimizer_2_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_2_sub->Dump_State("solution_goodOpt.txt");
                        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
                    }
                    rmse_good_v4[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_2_sub);
                    std::cout << "final rmse = " << rmse_good_v4[iter*ratio_size + rt] << std::endl;

                    delete optimizer_2;
#ifndef SWEEP_LAZIER_FACTOR
                    delete optimizer_2_pool;
#endif
                    delete optimizer_2_sub;
                    delete parse_loop_2;
                }

                // Refined good graph that takes the (oversized) covisible subgraph as selection pool
                if (do_good_v5) {
                    printf("\n\n --------------- Start Good Graph Optimization V5 ---------------\n\n");
                    // stage 2: good subgraph optimization
                    CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_2 = new CMyParseLoop(*optimizer_2);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_2)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_2->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_2->reserv_pos_.clear();
                    parse_loop_2->reserv_pos_.push_back(0);

#ifdef SWEEP_LAZIER_FACTOR
                    // simply copy the pointer
                    CBAOptimizer * optimizer_2_pool = optimizer_2;
#else
                    // Phase 1: reduce the size of candidate pool with covisibility
                    findCovisSubgraph(N, size_t(pool_ratio_vec[4] * float(M)), parse_loop_2);
                    optimizer_2->Show_Stats();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_pool = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_2->reserv_pos_, *optimizer_2_pool);
#endif
                    // NOTE
                    // reduce the graph to meet cardinality constr. M
                    // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
                    // assuming the g2o input is sorted in a certain way:
                    // ...
                    // all camera rows
                    // ...
                    // a map row
                    // all edges linked to the map row
                    // ...
                    std::vector<size_t> subgraph_in_pool, subgraph_in_orig;
                    subgraph_in_pool.push_back(0);
#ifdef SWEEP_LAZIER_FACTOR
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool, lazier_factor_vec[4]);
#else
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool);
#endif

                    // extract the sub-selected entries from pool
                    subgraph_in_orig.clear();
                    for (const auto idx : subgraph_in_pool) {
#ifdef SWEEP_LAZIER_FACTOR
                        subgraph_in_orig.push_back(idx);
#else
                        subgraph_in_orig.push_back(parse_loop_2->reserv_pos_[idx]);
#endif
                    }

                    sort(subgraph_in_orig.begin(), subgraph_in_orig.end());
                    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", subgraph_in_orig.size(), logdt);
                    printf("reserv_vtx_: ");
                    for (size_t i=0; i<subgraph_in_orig.size(); ++i)
                        printf("%d ", subgraph_in_orig[i]);
                    printf("\n");
                    optimizer_2_pool->Show_Stats();
                    timeCostSub_good_v5[iter*ratio_size + rt] = optimizer_2_pool->GetTotalTime();
                    logDet_good_raw_v5[iter*ratio_size + rt] = logdt;

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), subgraph_in_orig, *optimizer_2_sub);
                    graphScale_good_v5[iter*ratio_size + rt] = optimizer_2_sub->n_Vertex_Num();
                    //                    cout << optimizer_2_sub->m_lmk_count << " vs " << optimizer_2_sub->n_Vertex_Num() << endl;

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodInit.graph";
                        optimizer_2_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_good_ref_v5[iter*ratio_size + rt] = optimizer_2_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_good_ref_v5[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_2_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_2_sub->Show_Stats();

                    timeCostOpt_good_v5[iter*ratio_size + rt] = optimizer_2_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodOpt.graph";
                        optimizer_2_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_2_sub->Dump_State("solution_goodOpt.txt");
                        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
                    }
                    rmse_good_v5[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_2_sub);
                    std::cout << "final rmse = " << rmse_good_v5[iter*ratio_size + rt] << std::endl;

                    delete optimizer_2;
#ifndef SWEEP_LAZIER_FACTOR
                    delete optimizer_2_pool;
#endif
                    delete optimizer_2_sub;
                    delete parse_loop_2;
                }

                // Refined good graph that takes the (oversized) covisible subgraph as selection pool
                if (do_good_v6) {
                    printf("\n\n --------------- Start Good Graph Optimization V6 ---------------\n\n");
                    // stage 2: good subgraph optimization
                    CBAOptimizer * optimizer_2 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_2 = new CMyParseLoop(*optimizer_2);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_2)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_2->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_2->reserv_pos_.clear();
                    parse_loop_2->reserv_pos_.push_back(0);

#ifdef SWEEP_LAZIER_FACTOR
                    // simply copy the pointer
                    CBAOptimizer * optimizer_2_pool = optimizer_2;
#else
                    // Phase 1: reduce the size of candidate pool with covisibility
                    //                    findCovisSubgraph(N, size_t(pool_ratio_vec[5] * float(M)), parse_loop_2);
                    findCovisSubgraph(N, size_t(0), parse_loop_2);
                    optimizer_2->Show_Stats();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_pool = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_2->reserv_pos_, *optimizer_2_pool);
#endif
                    // NOTE
                    // reduce the graph to meet cardinality constr. M
                    // the returned subset is indexed w.r.t. camera only system (cam0 - 0, cam1 - 1, etc.)
                    // assuming the g2o input is sorted in a certain way:
                    // ...
                    // all camera rows
                    // ...
                    // a map row
                    // all edges linked to the map row
                    // ...
                    std::vector<size_t> subgraph_in_pool, subgraph_in_orig;
                    subgraph_in_pool.push_back(0);
#ifdef SWEEP_LAZIER_FACTOR
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool, lazier_factor_vec[5]);
#else
                    logdt = optimizer_2_pool->Find_Subgraph(M, 1, subgraph_in_pool);
#endif

                    // extract the sub-selected entries from pool
                    subgraph_in_orig.clear();
                    for (const auto idx : subgraph_in_pool) {
#ifdef SWEEP_LAZIER_FACTOR
                        subgraph_in_orig.push_back(idx);
#else
                        subgraph_in_orig.push_back(parse_loop_2->reserv_pos_[idx]);
#endif
                    }

                    sort(subgraph_in_orig.begin(), subgraph_in_orig.end());
                    printf("reserv_vtx_.size() = %d; logDet(S) = %.03f\n", subgraph_in_orig.size(), logdt);
                    printf("reserv_vtx_: ");
                    for (size_t i=0; i<subgraph_in_orig.size(); ++i)
                        printf("%d ", subgraph_in_orig[i]);
                    printf("\n");
                    optimizer_2_pool->Show_Stats();
                    timeCostSub_good_v6[iter*ratio_size + rt] = optimizer_2_pool->GetTotalTime();
                    logDet_good_raw_v6[iter*ratio_size + rt] = logdt;

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_2_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), subgraph_in_orig, *optimizer_2_sub);
                    graphScale_good_v6[iter*ratio_size + rt] = optimizer_2_sub->n_Vertex_Num();
                    //                    cout << optimizer_2_sub->m_lmk_count << " vs " << optimizer_2_sub->n_Vertex_Num() << endl;

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodInit.graph";
                        optimizer_2_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_good_ref_v6[iter*ratio_size + rt] = optimizer_2_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_good_ref_v6[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_2_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_2_sub->Show_Stats();

                    timeCostOpt_good_v6[iter*ratio_size + rt] = optimizer_2_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_goodOpt.graph";
                        optimizer_2_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_2_sub->Dump_State("solution_goodOpt.txt");
                        //        optimizer_2_sub->Plot3D("graph_goodOpt.tga");
                    }
                    rmse_good_v6[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_2_sub);
                    std::cout << "final rmse = " << rmse_good_v6[iter*ratio_size + rt] << std::endl;

                    delete optimizer_2;
#ifndef SWEEP_LAZIER_FACTOR
                    delete optimizer_2_pool;
#endif
                    delete optimizer_2_sub;
                    delete parse_loop_2;
                }

                if (do_covi) {
                    printf("\n\n --------------- Start Co-vis Graph Optimization ---------------\n\n");
                    // stage 3: co-vis weight
                    CBAOptimizer * optimizer_3 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_3 = new CMyParseLoop(*optimizer_3);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_3)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_3->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_3->reserv_pos_.clear();
                    parse_loop_3->reserv_pos_.push_back(0);

                    // reduce the graph to meet cardinality constr. M
                    // find the shared portion of edges between parse_loop_3.reserv_pos_[0] and other cameras
                    findCovisSubgraph(N, M, parse_loop_3);
                    optimizer_3->Show_Stats();
                    timeCostSub_covi[iter*ratio_size + rt] = optimizer_3->GetTotalTime();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_3_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_3->reserv_pos_, *optimizer_3_sub);
                    graphScale_covi[iter*ratio_size + rt] = optimizer_3_sub->n_Vertex_Num();

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_covisInit.graph";
                        optimizer_3_sub->Dump_Graph(fname_init.c_str());
                    }
                    logDet_covi[iter*ratio_size + rt] = optimizer_3_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_covi[iter*ratio_size + rt] << std::endl;

                    // optimize the system
                    optimizer_3_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_3_sub->Show_Stats();

                    timeCostOpt_covi[iter*ratio_size + rt] = optimizer_3_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_covisOpt.graph";
                        optimizer_3_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_3_sub->Dump_State("solution_covisOpt.txt");
                    }
                    rmse_covi[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_3_sub);
                    std::cout << "final rmse = " << rmse_covi[iter*ratio_size + rt] << std::endl;

                    delete optimizer_3;
                    delete optimizer_3_sub;
                    delete parse_loop_3;
                }

                if (do_rand) {
                    printf("\n\n --------------- Start Random Graph Optimization ---------------\n\n");
                    // stage 4: random
                    CBAOptimizer * optimizer_4 = new CBAOptimizer(b_verbose);
                    // create the optimizer object
                    CMyParseLoop * parse_loop_4 = new CMyParseLoop(*optimizer_4);
                    if(!CMyParser().Parse(fname_data.c_str(), *parse_loop_4)) {
                        fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
                        return -1;
                    }

                    N = parse_loop_4->poses_vtx_.size();
                    printf("total num. of camera poses = %d\n", N);
                    printf("target num. of camera poses = %d\n", M);
                    // set the last camera state un-removable
                    parse_loop_4->reserv_pos_.clear();
                    parse_loop_4->reserv_pos_.push_back(0);

                    // reduce the graph to meet cardinality constr. M
                    findRandSubgraph(N, M, parse_loop_4);
                    optimizer_4->Show_Stats();
                    timeCostSub_rand[iter*ratio_size + rt] = optimizer_4->GetTotalTime();

                    // re-config the optimizer with selected subgraph
                    CBAOptimizer * optimizer_4_sub = new CBAOptimizer(b_verbose);
                    BuildSubGraph(fname_data.c_str(), parse_loop_4->reserv_pos_, *optimizer_4_sub);
                    graphScale_rand[iter*ratio_size + rt] = optimizer_4_sub->n_Vertex_Num();

                    //
                    if (save_simu_data) {
                        std::string fname_init = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_randInit.graph";
                        optimizer_4_sub->Dump_Graph(fname_init.c_str());
                    }
                    // optimize the system
                    logDet_rand[iter*ratio_size + rt] = optimizer_4_sub->GetFullSystemLogDet();
                    std::cout << "init logDet = " << logDet_rand[iter*ratio_size + rt] << std::endl;

                    optimizer_4_sub->Optimize(OPT_ITER_NUM);
                    // show the timing, save the results
                    optimizer_4_sub->Show_Stats();

                    timeCostOpt_rand[iter*ratio_size + rt] = optimizer_4_sub->GetTotalTime();
                    if (save_simu_data) {
                        std::string fname_opt = std::string(p_s_input_file) + "/round_" + std::to_string(iter) + "_randOpt.graph";
                        optimizer_4_sub->Dump_Graph(fname_opt.c_str());
                        //            optimizer_4_sub->Dump_State("solution_randOpt.txt");
                    }
                    rmse_rand[iter*ratio_size + rt] = simulator_->getRMSE(optimizer_4_sub);
                    std::cout << "final rmse = " << rmse_rand[iter*ratio_size + rt] << std::endl;

                    delete optimizer_4;
                    delete optimizer_4_sub;
                    delete parse_loop_4;
                }
            }

            //
            delete simulator_;
        }

        // log results
        std::string fname_log;
        fname_log = std::string(p_s_input_file) + "/logSubTime_" + std::to_string(subgraph_scales[sc]) + ".txt";
        saveLog(fname_log.c_str(),
                timeCostSub_full,
                timeCostSub_good_v1,
                timeCostSub_good_v2,
                timeCostSub_good_v3,
                timeCostSub_good_v4,
                timeCostSub_good_v5,
                timeCostSub_good_v6,
                timeCostSub_covi,
                timeCostSub_rand,
                ratio_size);
        fname_log = std::string(p_s_input_file) + "/logOptTime_" + std::to_string(subgraph_scales[sc]) + ".txt";
        saveLog(fname_log.c_str(),
                timeCostOpt_full,
                timeCostOpt_good_v1,
                timeCostOpt_good_v2,
                timeCostOpt_good_v3,
                timeCostOpt_good_v4,
                timeCostOpt_good_v5,
                timeCostOpt_good_v6,
                timeCostOpt_covi,
                timeCostOpt_rand,
                ratio_size);
        fname_log = std::string(p_s_input_file) + "/logRMSE_" + std::to_string(subgraph_scales[sc]) + ".txt";
        saveLog(fname_log.c_str(),
                rmse_full,
                rmse_good_v1,
                rmse_good_v2,
                rmse_good_v3,
                rmse_good_v4,
                rmse_good_v5,
                rmse_good_v6,
                rmse_covi,
                rmse_rand,
                ratio_size);
        fname_log = std::string(p_s_input_file) + "/logGraphScale_" + std::to_string(subgraph_scales[sc]) + ".txt";
        saveLog(fname_log.c_str(),
                graphScale_full,
                graphScale_good_v1,
                graphScale_good_v2,
                graphScale_good_v3,
                graphScale_good_v4,
                graphScale_good_v5,
                graphScale_good_v6,
                graphScale_covi,
                graphScale_rand,
                ratio_size);
        fname_log = std::string(p_s_input_file) + "/logLogDetRaw_" + std::to_string(subgraph_scales[sc]) + ".txt";
        saveLog(fname_log.c_str(),
                logDet_full,
                logDet_good_raw_v1,
                logDet_good_raw_v2,
                logDet_good_raw_v3,
                logDet_good_raw_v4,
                logDet_good_raw_v5,
                logDet_good_raw_v6,
                logDet_covi,
                logDet_rand,
                ratio_size);
        fname_log = std::string(p_s_input_file) + "/logLogDetRef_" + std::to_string(subgraph_scales[sc]) + ".txt";
        saveLog(fname_log.c_str(),
                logDet_full,
                logDet_good_ref_v1,
                logDet_good_ref_v2,
                logDet_good_ref_v3,
                logDet_good_ref_v4,
                logDet_good_ref_v5,
                logDet_good_ref_v6,
                logDet_covi,
                logDet_rand,
                ratio_size);
    }

    //    printf("\n\n============= Summarized Time Log (ms) =============\n");
    //    printf("Subgraph Selection:           ");
    //    printf("%.01f ", timeCostSub_good / double(SIMU_ROUND) * 1000.0);
    //    printf("%.01f ", timeCostSub_covi / double(SIMU_ROUND) * 1000.0);
    //    printf("%.01f ", timeCostSub_rand / double(SIMU_ROUND) * 1000.0);
    //    printf("\n");
    //    printf("Subgraph Optimization: ");
    //    printf("%.01f ", timeCostOpt_full / double(SIMU_ROUND) * 1000.0);
    //    printf("%.01f ", timeCostOpt_good / double(SIMU_ROUND) * 1000.0);
    //    printf("%.01f ", timeCostOpt_covi / double(SIMU_ROUND) * 1000.0);
    //    printf("%.01f ", timeCostOpt_rand / double(SIMU_ROUND) * 1000.0);
    //    printf("\n");
}
#endif

/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int n_arg_num, const char **p_arg_list)
{
    bool b_verbose = true;
    const char *p_s_input_file = 0;
    for(int i = 1; i < n_arg_num; ++ i) {
        if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
            printf("use: good_graph_testbed [-i|--input <input-graph-file>] [-q|--quiet]\n");
            return 0;
        } else if(!strcmp(p_arg_list[i], "--quiet") || !strcmp(p_arg_list[i], "-q"))
            b_verbose = false;
        else if(i + 1 == n_arg_num) {
            fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
            return -1;
        } else if(!strcmp(p_arg_list[i], "--input") || !strcmp(p_arg_list[i], "-i"))
            p_s_input_file = p_arg_list[++ i];
        else {
            fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
            return -1;
        }
    }
    if(!p_s_input_file) {
        fprintf(stderr, "error: no input file specified. run with -h or --help to get help\n");
        return -1;
    }
    // "parse" cmdline

#ifdef EVAL_LAZIER_GREEDY

    testLazierGreedy(p_s_input_file, b_verbose);

#elif defined EVAL_SUBSET_METHODS_REAL

    testSubgraphBA_realData(p_s_input_file, b_verbose);

#elif defined EVAL_SUBSET_METHOD_SIMU

    testSubgraphBA_simulation(p_s_input_file, b_verbose);

#endif

    return 0;
}

/**
 *	@page baifaceexample Simple SLAM++ Interface Example
 *
 *	This example shows how to interface SLAM++ with other code, may that be C++ or even "C".
 *	It also demonstrates a very simple bundle adjustment (BA) optimizer, which takes graph files
 *	such as Venice (file venice871.g2o). The BA problem is posed using SE(3) global poses for
 *	cameras and global positions for points (rather than relative camera poses and points relative
 *	to camera).
 *
 *	The code is divided to two files, @ref Main.cpp and @ref BAOptimizer.cpp. All of SLAM++ heavy
 *	duty code is included only in BAOptimizer.cpp, while Main.cpp only includes the optimizer
 *	interface. This makes development of applications using SLAM++ easier, as Main.cpp now
 *	compiles in a few seconds (three seconds on a Core i7 laptop), while BAOptimizer.cpp takes
 *	a while to compile but needs to be rarely changed during the development.
 *
 *	We begin by creating an optimizer objecet:
 *
 *	@code
 *	CBAOptimizer optimizer
 *	@endcode
 *
 *	This object contains SLAM++ solver as well as the optimized system state. To fill it, one
 *	can call one of these functions:
 *
 * 	@code
 *	Eigen::Matrix<double, 11, 1> first_cam_pose, second_cam_pose;
 *	first_cam_pose << 0 0 0 0 0 0 500 500 0 0 0;
 *	optimizer.Add_CamVertex(0, first_cam_pose);
 *	second_cam_pose << 0 0 -10 0 0 0 500 500 0 0 0;
 *	optimizer.Add_CamVertex(1, second_cam_pose);
 *	// add a camera at the origin and another 10 units in front of it along z+
 *	// the optical centre is at (500, 500) pixels and the camera has no skew / distortion
 *	// (note that the poses are inverse - they transform points from world space to camera space)
 *
 *	optimizer.Add_XYZVertex(2, Eigen::Vector3d(10, 20, 30));
 *	// add a 3D point
 *
 *	optimizer.Add_P2C3DEdge(2, 0, Eigen::Vector2d(50, 50), Eigen::Matrix2d::Identity() * 10);
 *	optimizer.Add_P2C3DEdge(2, 1, Eigen::Vector2d(75, 75), Eigen::Matrix2d::Identity() * 10);
 *	// add an observation of the point by the two cameras, with covariance [10 0; 0 10]
 *	@endcode
 *
 *	We can access the state using:
 *
 * 	@code
 *	Eigen::Map<Eigen::VectorXd> camera_0_state = optimizer.r_Vertex_State(0);
 *	@endcode
 *
 *	Where the address of the vector data does not change over time (e.g. when more vertices are
 *	added, as would happen with std::vector elements).
 *	Now that we have the system ready, we can optimize using:
 *
 * 	@code
 *	optimizer.Optimize();
 *	@endcode
 *
 *	This also makes the value of camera_0_state change, so that the optimized camera pose
 *	is available. Alternately, we can save the results using:
 *
 * 	@code
 *	Dump_State("solution.txt");
 *	@endcode
 *
 *	It is also possible to save the optimized graph, like this:
 *
 * 	@code
 *	optimizer.Dump_Graph("solution.graph");
 *	@endcode
 *
 *	This is especially handy if doing processing from sensorial data - other researchers can
 *	then use this graph file without having to process the sensor data again. Note that this
 *	graph does not preserve the order in which the edges and vertices were added to the optimizer.
 *	It outputs all the vertices first (ordered by their indices), then outputs the edges (in the
 *	order they were introduced to the optimizer).
 *
 *	It is now possible to create a statically linked library which you can link to your existing
 *	code easily, and just use the simple optimizer interface where no templates are involved.
 *	@ref BAOptimizer.h also features a simple "C" interface which can be easily extended with
 *	additional functionality.
 *
 */

/*
 *	end-of-file
 */
