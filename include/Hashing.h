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

/**
* This file is developed based on MILD.
*
* Copyright (C) 2017 Lei Han and Lu Fang <lhanaf@connect.ust.hk; eefang@ust.hk> (Hong Kong University of Science and Technology)
* For more information see <https://github.com/lhanaf/MILD>
*
* Related papers:
* [1] Multi-Index Hashing for Loop closure Detection. International Conference on Multimedia Expo, 2017. Best Student Paper Awards.
* [2] Beyond SIFT Using Binary features in Loop Closure Detection. IROS 2017.
*/

#ifndef HASHING_H
#define HASHING_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <list>
#include <opencv2/core/core.hpp>
// NOTE
// commented out in map hash branch; to be verified
//#include <boost/thread.hpp>
#include <mutex>
#include <deque>

#include "MapPoint.h"


/* =========================    HASHING   ========================= */

/* --- options of hashing config; DO NOT EDIT unless you know what you are doing! --- */
#define MAX_SUPPORTED_BIT_LENGTH    32 /*(sizeof(size_t))*/
#define MAX_REPEATED_FRAME_NUM      1
#define MIN_BUCKET_SIZE             1
#define MAX_BUCKET_SIZE             20 // 10 // 200 //
#define NUM_CANDIDATE_FEATURES      20 // 10 // 1 //
#define NUM_ADDITIONAL_FEATURES     0 // 1 // 3 // 2   // 0 if use all features in the bucket
#define NUM_TOTAL_HASHTABLES        32
#define NUM_ACTIVE_HASHTABLES       8 // 32 // 2 // 12 // 1 // 
//#define NUM_RETRIEVE_CTRL
//#define NUM_RETRIEVE_TOP_N          10


/* --- options of debuggin and print out --- */
//#define MAPHASH_VERBOSE
//#define MAPHASH_OTS_VERBOSE
//#define DRAW_ADDITIONAL_STATISTICS


/* --- options of parameters used in online table selection --- */
#define MIN_DETERMINE_THRESHOLD     80
#define MIN_NUM_HASHTABLES          1
#define MAX_NUM_HASHTABLES          24 // 16 // 
#define MAX_TRACK_LOCALMAP_TIME     0.020 // 0.012 // 
#define MIN_TIME_DIFF               0.001



/* --- obselete --- */
//#define COMBINE_BASELINE_TRACK_LOCALMAP
//#define DISABLE_LOCALMAP_TRACKING
//#define ENABLE_ERROR_HANDLING
//#define DELAYED_MAP_MATCHING_USING_HASHING
//#define SAVE_PUB_FRAME
//#define NUM_RANDOM_SAMPLES 500


/* ================================================================ */




//#define HASHING_WARNING

namespace HASHING
{

typedef ORB_SLAM2::MapPoint* Entry;

//typedef std::vector<Entry> Bucket;

bool cmpEntries(Entry a, Entry b);

class Bucket{

public:
    Bucket();
    ~Bucket();
    // insert entry into the bucket
    void add(const Entry e);
    // retrieve entries from the bucket
    std::vector<Entry> back(int n = 1);
    int back(std::vector<Entry> &dst, int n=1);
    // get the latest added entry
    Entry front();

    //        size_t size();
    //        Entry at(int i);
    //        void add(const vector<Entry> &es);
    //        template <typename T>
    //        void prepend(const std::deque<Entry> &es);

private:
    std::deque<Entry> mEntries;
    std::deque<Entry>::iterator mIter;

    Entry mLatest;

//    boost::mutex mMutexBucket;
//    boost::mutex mMutexLatest;

    std::mutex mMutexLatest;
    std::mutex mMutexBucket;
    //    std::vector<Entry> entries;
    //    int p_next;

};

//typedef std::vector<Bucket> HTable;
//typedef std::vector<HTable> HTableSet;

//class HashTableSet
//{
//public:
//    HashTableSet(const size_t table_size);
//    //    Bucket* getBucket(const size_t bucket_id);
//    //    Entry getRecentEntry(const size_t bucket_id, const unsigned int n = 1);
//    //    unsigned int getBucketSize(const size_t bucket_id);
//    //    void reserve(const size_t nBuckets, const unsigned int nEntries = MAX_BUCKET_SIZE);
//    //    void clearSpace();
//    void addEntry(const int table_id, const size_t bucket_id, const Entry feature);

//        void getEntries(const size_t bucket_id, std::vector<Entry>& entries, const unsigned int n = 1);

//    //    inline unsigned int getHashTableSize();
//    //    void assignID(const int id);
//private:
//        std::vector<vector<Bucket*> > mvvpBuckets;
//    //    int HashTableID;
//    //        boost::mutex mMutexBucket;

//    // # Hash Tables
//    // ### Hash Table
//    // ##### Bucket
//    // ####### Entry
//    //    std::vector<std::vector<std::deque<Entry>::iterator > > iters;
//    //    std::vector<std::vector<std::deque<Entry> > > entries;

//    //    std::mutex mMutexIter;
//};

class MultiIndexHashing
{
public:
    MultiIndexHashing(unsigned int feature_length = 256, unsigned int table_num = 16, /*unsigned int nactives = 6,*/
                      unsigned int max_num_per_bucket = MAX_BUCKET_SIZE, std::string logfile_path = "./MultiIndexHashing_details.log");

    MultiIndexHashing(){}
    ~MultiIndexHashing(){}

    void initialize();
    inline void hash(const unsigned int* pdesc, std::vector<size_t>& bucket_idx);
    bool insert(const Entry feature);

    //    void query(const Entry feature, std::vector<Entry>& candidates);
    //    void query(const cv::Mat& mDesc, std::vector<Entry>& candidates, unsigned int nactive_tables = NULL, unsigned int nretrieved = INT_MAX);
    //    void query(const unsigned int* pDesc, std::vector<Entry>& candidates, const std::vector<unsigned int> &active_tables, unsigned int nretrieved = INT_MAX);

    void query(const unsigned int* pDesc, std::vector<Entry>& candidates, unsigned int nactive_tables = NULL, unsigned int nretrieved = INT_MAX);
//    void query(const unsigned int* pDesc, std::vector<Entry>& candidates, std::vector<Entry>& addi_candidates,
//               unsigned int nactive_tables, unsigned int nretrieved, unsigned int addi_nretrieved);
    //    void query(const unsigned int* pDesc, std::vector<bool> &active_hashtables, std::vector<Entry>& candidates, std::vector<Entry>& addi_candidates);
    void query(const unsigned int* pDesc, std::vector<Entry>& candidates, std::vector<Entry>& addi_candidates, std::vector<int> &addi_nfeatures);

    //    // for debuging
    //    void printBuckets(const unsigned int* pDesc, unsigned int nactive_tables = NULL);
    unsigned int getSubstringLen();
    int updateDynamics(double e);

private:
    unsigned int feature_length;
    unsigned int hashtable_num;
    unsigned int max_feature_num_per_bucket;
    unsigned int bits_per_substring;
    size_t bucket_num_per_hashtable;

    std::vector<std::vector<Bucket> > mvvBuckets;
    std::string logfile;
    int dynamics;

    //    std::mutex mMutexUpdate;
};
}

#endif // HASHING_H
