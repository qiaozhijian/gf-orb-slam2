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

#include "Hashing.h"

#include <bitset>

//static void outbin(unsigned int x)
//{
//    cout<<bitset<sizeof(unsigned int)*8>(x)<<endl;
//}

namespace HASHING
{


bool cmpEntries(Entry a, Entry b)
{
    return (a->mnQueriedScore > b->mnQueriedScore); //descending
}

//template<typename FeatureType>
void MultiIndexHashing/*<FeatureType>*/::hash(const unsigned int* pdesc, std::vector<size_t>& bucket_idx)
{
    //    const unsigned int* pdesc = descriptor.ptr<uint32_t>();

    if(bits_per_substring == 8)
    {
        unsigned char * entry_value = (unsigned char *)pdesc;
        for(int i=0; i< hashtable_num; i++)
        {
            bucket_idx[i] = entry_value[i];
        }
    }
    else if(bits_per_substring == 16)
    {
        unsigned short * entry_value = (unsigned short *)pdesc;
        for(int i=0; i< hashtable_num; i++)
        {
            bucket_idx[i] = entry_value[i];
        }
    }
    else
    {
        size_t index = 0;
        size_t mask[MAX_SUPPORTED_BIT_LENGTH];

        // generate mask
        for (int j = 0; j < MAX_SUPPORTED_BIT_LENGTH; j++){
            mask[j] = 0;
            for(int i = 0; i < j; i++)
                mask[j] += 1 << i;
        }

        for(int i = 0; i < hashtable_num; i++){
            // check how many bytes a substring occupies
            int si = (bits_per_substring * i) / MAX_SUPPORTED_BIT_LENGTH;
            int ei = (bits_per_substring * (i + 1)) / MAX_SUPPORTED_BIT_LENGTH;
            int sp = (bits_per_substring * i) % MAX_SUPPORTED_BIT_LENGTH;
            int ep = (bits_per_substring * (i + 1)) % MAX_SUPPORTED_BIT_LENGTH;

            if(si == ei){   // completely contained in a sizeof(size_t)
                index = (pdesc[si] >> sp) & mask[bits_per_substring];
            }
            if(si < ei){    // cross two sizeof(size_t)
                index = ((pdesc[si] >> sp) & mask[MAX_SUPPORTED_BIT_LENGTH - sp]) +
                        ((pdesc[ei] & mask[ep]) << (bits_per_substring - ep));
            }

            bucket_idx[i] = index;
        }
    }
}

Bucket::Bucket()
{
    //    p_next = 0;
    //    entries.reserve(MAX_BUCKET_SIZE);
    //    if (entries.size() > 0)
    //        iter = entries.begin();
    //    else
    //        iter = NULL;
    mEntries.clear();
    //    mLatest = NULL;
    mLatest = static_cast<Entry>(NULL);
}

Bucket::~Bucket(){}

void Bucket::add(const Entry e/*, const int HashTableID*/)
{
    //    boost::mutex::scoped_lock lock(mMutexEntries);
    //    std::unique_lock<mutex> lock(mMutexEntries);
    //    if(entries.size() < MAX_BUCKET_SIZE)
    //        entries.push_back(e);
    //    else
    //    {
    //        entries[p_next]->mvbHashed[HashTableID] = false;
    //        //        cout<<"hashtable id:"<<HashTableID<<", turn false, bucket size:"<<entries.size()<<endl;
    //        entries[p_next] = e;
    //    }

    //    p_next = (p_next + 1) % MAX_BUCKET_SIZE;

    // push item to the front
    mEntries.push_front(e);

    {
        // update current iterator
        //        boost::mutex::scoped_lock lock1(mMutexBucket);
        std::unique_lock<std::mutex> lock1(mMutexBucket);
        mIter = mEntries.begin();
    }

    //    if (entries.size() > MAX_BUCKET_SIZE){
    //        Entry ent = entries.front();
    //        ent->mvbHashed[HashTableID] = false;
    //        entries.pop_front();
    //    }
    {
        std::unique_lock<std::mutex> lock2(mMutexLatest);
        mLatest = e;
    }
}

//void Bucket::add(const vector<Entry> &es)
//{
//    for(int i=0; i!=es.size(); ++i)
//        entries.push_front(es[i]);

//    {
//        std::unique_lock<mutex> lock1(mMutexBucket);
//        iter = entries.begin();
//    }

//    {
//        std::unique_lock<mutex> lock2(mMutexLatest);
//        latest = es.back();
//    }
//}

//template <typename T>
//void Bucket::prepend(const T<Entry> &es)
//{
//    //    void insert(iterator it,const_iterator first,const_iterator last):
//    mEntries.insert(mEntries.begin(), es.begin(), es.end());

//    {
//        std::unique_lock<mutex> lock1(mMutexBucket);
//        mIter = mEntries.begin();
//    }

//    {
//        std::unique_lock<mutex> lock2(mMutexLatest);
//        mLatest = es.front();
//    }
//}

std::vector<Entry> Bucket::back(int n)
{
    //    boost::mutex::scoped_lock lock(mMutexEntries);
    //    std::unique_lock<mutex> lock(mMutexEntries);

    std::vector<Entry> r;

    if(n == 0)
        return r;

    std::deque<Entry>::iterator it;
    int s;

    {
        std::unique_lock<std::mutex> lock(mMutexBucket);
        it = mIter; //clone current iterator
        s = mEntries.size();
    }

    if(s == 0)
        return r;

    //    //    int p_curr = (p_next - 1) % s;
    //    //    while(p_curr<0)
    //    //        p_curr += s;

    //    //    p_curr = p_curr + 1;

    if(n >= s)
        r.insert(r.end(), it, mEntries.end());
    //#ifdef HASHING_WARNING
    //        std::cout << "HASHING-WARNING: .back() bucket size is less than the required return number!" << std::endl;
    //#endif
    else
        r.insert(r.end(), it, it+n);

    //    //    if(s == MAX_BUCKET_SIZE){
    //    //        if(n <= p_curr)
    //    //            r.insert(r.end(), entries.begin() + p_curr - n, entries.begin() + p_curr);

    //    //        if(n > p_curr){
    //    //            r.insert(r.end(), entries.begin(), entries.begin() + p_curr);
    //    //            r.insert(r.end(), entries.end() - n + p_curr, entries.end());
    //    //        }
    //    //    }

    //    //    if(s < MAX_BUCKET_SIZE)
    //    //        r.insert(r.end(), entries.end() - n, entries.end());

    //    //    if(s > MAX_BUCKET_SIZE){
    //    //        std::cerr << "ERROR: .back() bucket size is over the max limit!" << std::endl;
    //    //        exit(-1);
    //    //    }

    //    //    deque<Entry>::reverse_iterator rit = entries.rbegin();
    //    //    for (; rit != entries.rbegin()+n /*entries.rend()*/; ++rit)
    //    //        r.push_back(*rit);

    return r;
}

int Bucket::back(std::vector<Entry> &dst, int n)
{
    dst.clear();

    if(n == 0)
        return dst.size();

    std::deque<Entry>::iterator it;
    int s;

    {
        std::unique_lock<std::mutex> lock(mMutexBucket);
        it = mIter; //clone current iterator
        s = mEntries.size();
    }

    if(s == 0)
        return dst.size();

    if(n >= s)
        dst.insert(dst.end(), it, mEntries.end());
    //#ifdef HASHING_WARNING
    //        std::cout << "HASHING-WARNING: .back() bucket size is less than the required return number!" << std::endl;
    //#endif
    else
        dst.insert(dst.end(), it, it+n);

    return dst.size();
}

//size_t Bucket::size()
//{
//    //    boost::mutex::scoped_lock lock(mMutexEntries);
//    std::unique_lock<mutex> lock(mMutexEntries);

//    return entries.size();
//}

//Entry Bucket::at(int i)
//{
//    //    boost::mutex::scoped_lock lock(mMutexEntries);
////    std::unique_lock<mutex> lock(mMutexEntries);
//    if (i<0)
//        return NULL;

//    int s;
//    {
//        std::unique_lock<mutex> lock(mMutexIter);
//        s= entries.size();
//    }

//    if(s==0)
//        return NULL;

//    if(i > s-1){
//        i = s-1;
////#ifdef HASHING_WARNING
////        std::cout << "Hashing Warning: index is out of range at func .at() !!!" << std::endl;
////#endif
//    }

//    if(s < MAX_BUCKET_SIZE)
//        return entries[i];
//    else{
//        int k = (i + p_next) % MAX_BUCKET_SIZE;
//        return entries[k];
//    }
//}

Entry Bucket::front(/*int i, bool circle*/)
{
    //    boost::mutex::scoped_lock lock(mMutexEntries);
    //    std::unique_lock<mutex> lock(mMutexEntries);



    //    int s = entries.size();

    //    if(s==0 || i<0)
    //        return NULL;

    //    if(i > s-1){
    //        i = s-1;
    //#ifdef HASHING_WARNING
    //        std::cout << "Hashing Warning: index is out of range at func .at_last() !!!" << std::endl;
    //#endif
    //    }

    //    if(s < MAX_BUCKET_SIZE){
    //        int k = s - i - 1;
    //        return entries[k];
    //    }
    //    else{
    //        int k = p_next - i - 1;
    //        while(k<0)
    //            k += MAX_BUCKET_SIZE;
    //        return entries[k];
    //    }
    std::unique_lock<std::mutex> lock(mMutexLatest);

    return mLatest;
}

//void HashTable::getEntries(const size_t bucket_id, std::vector<Entry>& out, const unsigned int n)
//{
//    out.clear();
//    //    boost::mutex::scoped_lock lock(mMutexBucket);
//    Bucket* pB = pBuckets[bucket_id];
//    //    unsigned int nentries = b.size();
//    //    int s = std::min(n, nentries);
//    //    //        //        recent_size < entries.size() ? recent_size : entries.size();
//    //    //        std::cout<<"ready to getEntries..."<<std::endl;
//    //    if (s >= 0 /*&& entries.size() >= recent_size*/)
//    //    {
//    //        out.insert(out.end(), b.end()-s, b.end());
//    //        //        for(int i = nentries-s; i!=nentries; i++)
//    //        //        {
//    //        //            out.push_back(entries[i]->GetPtr());
//    //        //        }
//    //    }
//    //    //        std::cout<<"getEntries done..."<<std::endl;

//    out = pB->back(n);
//}

//void HashTable::assignID(const int id)
//{
//    this->HashTableID = id;
//}

//unsigned int Bucket::getBucketSize()
//{
//    boost::mutex::scoped_lock lock(mMutexEntries);
//    return entries.size();
//}

//Entry Bucket::getLastOne()
//{
//    boost::mutex::scoped_lock lock(mMutexEntries);
//    return entries.back();
//}

//unsigned int Bucket::getPreviousElementId(const unsigned int n)
//{
//    boost::mutex::scoped_lock lock(mMutexEntries);
//    unsigned int m = entries.size();
//    if (m>=n)
//        return entries[m-n]->mnId;
//    else
//        return NULL;
//}

//void Bucket::addEntry(const Entry feature)
//{
//    boost::mutex::scoped_lock lock(mMutexEntries);
//    entries.push_back(feature->GetPtr());
//    //    entries.push_back(feature);
//    //    std::cout<<entries.size()<<std::endl;
//}

//void Bucket::clearSpace()
//{
//    entries.clear();
//}


// class HashTable
//template<typename FeatureType>
//void HashTable/*<FeatureType>*/::getEntries(const size_t bucket_id, std::vector<Entry> &entries, const unsigned int recent)
//{
//    buckets[bucket_id].getEntries(entries, recent);
//}

//Bucket* HashTable::getBucket(const size_t bucket_id)
//{
////    boost::mutex::scoped_lock lock(mMutexBucket);
//    return pBuckets[bucket_id];
//}

//Entry HashTable::getRecentEntry(const size_t bucket_id, const unsigned int n)
//{
//    //    boost::mutex::scoped_lock lock(mMutexBucket);
//    //    unsigned int m = buckets[bucket_id].size();
//    //    if (m>=n)
//    //        return buckets[bucket_id][m-n];
//    //    else
//    //        return NULL;
//    return pBuckets[bucket_id]->at_last(n);
//}

//size_t HashTable::getBucketSize()
//{
//    return buckets.size();
//}

//unsigned int HashTable::getBucketSize(const size_t bucket_id)
//{
//    //    boost::mutex::scoped_lock lock(mMutexBucket);
//    return pBuckets[bucket_id]->size();
//}

//template<typename FeatureType>
//void HashTable/*<FeatureType>*/::reserve(const size_t nBuckets, const unsigned int nEntries)

//{
//    // TODO: reserve nEntries space for each Bucket
//    pBuckets = std::vector<Bucket*>(nBuckets);
//    for(auto it=pBuckets.begin(); it!=pBuckets.end(); it++)
//        *it = new Bucket();
//}

////template<typename FeatureType>
//void HashTable/*<FeatureType>*/::clearSpace()
//{
//    for(auto it=buckets.begin(); it!= buckets.end(); it++)
//        it->clear();
//    //    buckets.clear();
//}

//void HashTableSet::addEntry(const int table_id, const size_t bucket_id, const Entry feature)
//{
//    //    boost::mutex::scoped_lock lock(mMutexBucket);
////        pBuckets[bucket_id]->add(feature, HashTableID);
//    //    cout<<"add feature to hashtable: "<<HashTableID<<endl;
////        buckets[bucket_id].addEntry(feature);
//    mvvpBuckets[table_id][bucket_id]->add(feature);
//}

//unsigned int HashTable::getHashTableSize()
//{
//    unsigned int s = 0;
//    for(auto it=buckets.begin();it!=buckets.end();it++)
//        s += it->getBucketSize();
//    return s;
//}


// class MultiIndexHashing
//template<typename FeatureType>
MultiIndexHashing/*<FeatureType>*/::MultiIndexHashing(unsigned int feature_length, unsigned int table_num, unsigned int max_feature_num_per_bucket, std::string logfile_path):
    feature_length(feature_length), hashtable_num(table_num), max_feature_num_per_bucket(max_feature_num_per_bucket), logfile(logfile_path), dynamics(0)
{
    bits_per_substring = feature_length / hashtable_num;

    if(bits_per_substring > sizeof(size_t) * 8 / 2){    // Note: support 32-bits substring at most
        std::cerr<<"Error: The substring length is too long to be supported! (32-bits at most)"<<std::endl;
        exit(-1);
    }

    //    mvvpBuckets = HTableSet(hashtable_num);
    //    for(int i=0; i!=hashtable_num; i++)
    //        hashtables[i].assignID(i);

    bucket_num_per_hashtable = static_cast<size_t>(pow(2, bits_per_substring));
    //    // allocate memory for hashtables
    //    for(auto it=hashtables.begin();it!=hashtables.end();it++)
    //    {
    //        it->reserve(bucket_num_per_hashtable);
    //        // clear the space
    //        //        it->clearSpace();
    //    }

    initialize();


    cout << endl << "Hashing Parameters: " << endl;
    cout << "- Number of Binary Bits: " << feature_length << endl;
    cout << "- Number of Hash Tables: " << table_num << endl;
    cout << "- Number of Bits per Substring: " << bits_per_substring << endl;
    cout << "- Max. Number of Entries per Bucket: " << max_feature_num_per_bucket << endl << endl;
}

void MultiIndexHashing::initialize()
{
    mvvBuckets.clear();
    mvvBuckets.resize(hashtable_num);
    for(int i=0; i!=hashtable_num; ++i){
        mvvBuckets[i] = std::vector<Bucket>(bucket_num_per_hashtable);
        //        cerr<<mvvBuckets[i].size()<<endl;
    }
}

//template<typename FeatureType>
bool MultiIndexHashing::insert(const Entry feature)
{
    //    std::unique_lock<mutex> lock(mMutexUpdate);

    bool insert_success = false;

    std::vector<size_t> bucket_idx(hashtable_num);

    cv::Mat mDesc = feature->GetDescriptor();
    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???

    // multi-index hash
    hash(desc, bucket_idx);

    //    /*-----------------------------TEST--------------------------------*/
    //    cv::Mat mdesc = feature->GetDescriptor();
    //    std::cout<<"Size of Descriptor:\t"<<mdesc.rows<<" x "<< mdesc.cols<<std::endl;
    //    std::cout<<mdesc<<std::endl;

    //    unsigned int* pdata = mdesc.ptr<unsigned int>();
    //    unsigned short* pdata_short = (unsigned short*)pdata;

    //    unsigned short* pdesc = (unsigned short*)desc;
    //    for(int i=0; i!= 16; i++)
    //    {
    //        if(i!=hashtable_num-1)
    //            std::cout<</*pdata[i]<<","<<*/bucket_idx[i]<<std::endl;
    //        else
    //            std::cout<<bucket_idx[i]<<std::endl;
    //    }
    //    exit(-1);
    //    /*-----------------------------------------------------------------*/

    //    candidates_list.clear();

    // duplicate current feature
    for(int hashtable_id = 0; hashtable_id < hashtable_num; hashtable_id++){
        // in case the same feature inserted multiple times into the same bucket

        if(!feature->mvbHashed[hashtable_id]){

            feature->mvbHashed[hashtable_id]=true;

            size_t bucket_id = bucket_idx[hashtable_id];

            //        // query the features stored in the same entry
            //        entry entry_items = features_buffer[hashtable_index][entry_index];

            //        // merge features in all the entries
            //        for(auto it=entry_items.begin(); it!=entry_items.end(); it++)
            //            candidates_list.insert(*it);

            Entry latest = mvvBuckets[hashtable_id][bucket_id].front();

            if(latest == NULL || latest->mnId != feature->mnId)
            {
                insert_success = true;
                mvvBuckets[hashtable_id][bucket_id].add(feature);
            }
        }

        //    return candidates_list.size();
    }

    return insert_success;
}

//void MultiIndexHashing::query(const cv::Mat &mDesc, std::vector<Entry> &candidates, unsigned int nactive_tables, unsigned int nretrieves)
//{
//    std::vector<size_t> bucket_idx(hashtable_num);

//    if(nactive_tables == NULL)
//        nactive_tables = hashtable_num;

//    std::vector<unsigned int> active_hashtables(nactive_tables);
//    std::iota(active_hashtables.begin(), active_hashtables.end(), 0);
//    //    cv::Mat mDesc = feature->GetDescriptor();
//    //    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???
//    //    const unsigned int* desc=pDesc;
//    // multi-index hash
//    candidates.clear();
//    for(int i = 0; i!=mDesc.rows; i++){
//        const unsigned int* pDesc = mDesc.ptr<unsigned int>(i);

//        hash(pDesc, bucket_idx);

//        // duplicate current feature
//        for(auto hashtable_id : active_hashtables){

//            size_t bucket_id = bucket_idx[hashtable_id];
//            // query the features stored in the same entry
//            std::vector<Entry> entries;
//            hashtables[hashtable_id].getEntries(bucket_id, entries, nretrieves);
//            // merge features in all the entries
//            for(auto it=entries.begin(); it!=entries.end(); it++)
//                candidates.push_back(*it);
//        }
//    }
//}

//void MultiIndexHashing::query(const Entry feature, std::vector<Entry> &candidates)
//{
//    std::vector<size_t> bucket_idx(hashtable_num);

//    cv::Mat mDesc = feature->GetDescriptor();
//    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???

//    // multi-index hash
//    hash(desc, bucket_idx);

//    candidates.clear();

//    // duplicate current feature
//    for(int hashtable_id = 0; hashtable_id < hashtable_num; hashtable_id++){

//        size_t bucket_id = bucket_idx[hashtable_id];

//        // query the features stored in the same entry
//        std::vector<Entry> entries;
//        hashtables[hashtable_id].getEntries(bucket_id, entries, NUM_CANDIDATE_FEATURES);

//        // merge features in all the entries
//        for(auto it=entries.begin(); it!=entries.end(); it++)
//            candidates.push_back(*it);
//    }

//    // remove duplicates
//    //    std::sort(candidates.begin(),candidates.end());
//    //    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

//    //    return candidates_list.size();
//}

//void MultiIndexHashing::query(const unsigned int *pDesc, std::vector<Entry> &candidates, const std::vector<unsigned int> &active_hashtables, unsigned int nretrieves)
//{
//    //    std::vector<size_t> bucket_idx(hashtable_num);

//    //    //    std::vector<unsigned int> hashtable_indices(nactives);
//    //    if(active_hashtables.size() == 0){
//    //        return;
//    //        //        active_hashtables = std::vector<unsigned int>(hashtable_num);
//    //        //        std::iota(active_hashtables.begin(), active_hashtables.end(), 0);
//    //    }
//    //    //    cv::Mat mDesc = feature->GetDescriptor();
//    //    //    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???
//    //    //    const unsigned int* desc=pDesc;
//    //    // multi-index hash
//    //    hash(pDesc, bucket_idx);

//    //    candidates.clear();
//    //    // duplicate current feature
//    //    for(auto hashtable_id : active_hashtables){

//    //        size_t bucket_id = bucket_idx[hashtable_id];
//    //        //        cout<<"4"<<endl;
//    //        // query the features stored in the same entry
//    //        std::vector<Entry> entries;
//    //        hashtables[hashtable_id].getEntries(bucket_id, entries, nretrieves);
//    //        //        cout<<"5"<<endl;
//    //        // merge features in all the entries
//    //        for(auto it=entries.begin(); it!=entries.end(); it++)
//    //            candidates.push_back(*it);
//    //        //        cout<<"6"<<endl;
//    //    }
//}

void MultiIndexHashing::query(const unsigned int *pDesc, std::vector<Entry> &candidates, unsigned int nactive_tables_top, unsigned int nretrieves)
{
    std::vector<size_t> bucket_idx(hashtable_num);
    if(nactive_tables_top == NULL)
        nactive_tables_top = hashtable_num;
    std::vector<unsigned int> active_hashtables(nactive_tables_top);
    std::iota(active_hashtables.begin(), active_hashtables.end(), 0);
    //    cv::Mat mDesc = feature->GetDescriptor();
    //    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???
    //    const unsigned int* desc=pDesc;
    // multi-index hash
    hash(pDesc, bucket_idx);

    candidates.clear();
    candidates.reserve(nactive_tables_top * nretrieves);
    //    vector<int> scores;
    //    scores.reserve(nactive_tables_top * nretrieves);
    //    vector<int> indices;
    //    indices.reserve(nactive_tables_top * nretrieves);

    // duplicate current feature
    for(auto hashtable_id : active_hashtables){

        size_t bucket_id = bucket_idx[hashtable_id];
        // query the features stored in the same entry
        std::vector<Entry> entries;
        mvvBuckets[hashtable_id][bucket_id].back(entries, nretrieves + dynamics);

        //        for (auto it=entries.begin(), itend=entries.end(); it!=itend; ++it){

        //            Entry e = *it;

        //            if (!e || e->isBad())
        //                continue;

        //            if (e->mnIdCandidates < 0){
        //                e->mnIdCandidates = candidates.size();
        //                candidates.push_back(e);

        //            }
        //            else
        //            {
        //                ++candidates[e->mnIdCandidates]->mnQueriedScore;
        //            }

        //        }

        candidates.insert(candidates.end(), entries.begin(), entries.end());
    }

    //    // sort
    //    if (candidates.size() > NUM_RETRIEVE_TOP_N){
    //        std::partial_sort (candidates.begin(), candidates.begin()+NUM_RETRIEVE_TOP_N, candidates.end(), HASHING::cmpEntries);
    ////        std::sort(candidates.begin(), candidates.end(), HASHING::cmpEntries);

    //        for(int i=0, ie = candidates.size(); i!=ie; ++i){
    //            candidates[i]->mnIdCandidates = -1;
    //            candidates[i]->mnQueriedScore = 0;
    //        }

    //        candidates.resize(NUM_RETRIEVE_TOP_N);
    //    }
    //    else
    //    {
    //        for(int i=0, ie = candidates.size(); i!=ie; ++i){
    //            candidates[i]->mnIdCandidates = -1;
    //            candidates[i]->mnQueriedScore = 0;
    //        }
    //    }

}

//void MultiIndexHashing::query(const unsigned int *pDesc, std::vector<Entry> &candidates, std::vector<Entry> &addi_candidates,
//                              unsigned int nactive_tables, unsigned int nretrieved, unsigned int addi_nretrieved)
//{
//    std::vector<size_t> bucket_idx(hashtable_num);
//    if(nactive_tables == NULL)
//        nactive_tables = hashtable_num;
//    std::vector<unsigned int> active_hashtables(nactive_tables);
//    std::iota(active_hashtables.begin(), active_hashtables.end(), 0);
//    //    cv::Mat mDesc = feature->GetDescriptor();
//    //    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???
//    //    const unsigned int* desc=pDesc;
//    // multi-index hash
//    hash(pDesc, bucket_idx);
//    candidates.clear();
//    addi_candidates.clear();

//    // duplicate current feature
//    for(auto hashtable_id : active_hashtables){

//        size_t bucket_id = bucket_idx[hashtable_id];
//        // query the features stored in the same entry
//        std::vector<Entry> entries;
//        mvvBuckets[hashtable_id][bucket_id].back(entries, nretrieved+addi_nretrieved);

//        // merge features in all the entries
//        //        for(auto it=entries.begin(); it!=entries.end(); it++)
//        //            candidates.push_back(*it);
//        if(entries.size() <= nretrieved){
//            candidates.insert(candidates.end(), entries.begin(), entries.end());
//        }
//        else{
//            candidates.insert(candidates.end(), entries.end()-nretrieved, entries.end());
//            addi_candidates.insert(addi_candidates.end(), entries.begin(), entries.end()-nretrieved);
//        }
//    }
//}

//void MultiIndexHashing::query(const unsigned int *pDesc, std::vector<bool> &active_hashtables, std::vector<Entry> &candidates, std::vector<Entry> &addi_candidates)
//{
//    std::vector<size_t> bucket_idx(hashtable_num);
//    //    if(nactive_tables == NULL)
//    //        nactive_tables = hashtable_num;
//    //    std::vector<unsigned int> active_hashtables(nactive_tables);
//    //    std::iota(active_hashtables.begin(), active_hashtables.end(), 0);
//    //    cv::Mat mDesc = feature->GetDescriptor();
//    //    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???
//    //    const unsigned int* desc=pDesc;
//    // multi-index hash
//    hash(pDesc, bucket_idx);
//    candidates.clear();
//    addi_candidates.clear();

//    // duplicate current feature
//    for(int k=0; k!=hashtable_num; k++){

//        size_t bucket_id = bucket_idx[k];
//        // query the features stored in the same entry
//        std::vector<Entry> entries;
//        hashtables[k].getEntries(bucket_id, entries, NUM_CANDIDATE_FEATURES);

//        // merge features in all the entries
//        //        for(auto it=entries.begin(); it!=entries.end(); it++)
//        //            candidates.push_back(*it);
//        //        if(entries.size() <= nretrieved){
//        //            candidates.insert(candidates.end(), entries.begin(), entries.end());
//        //        }
//        //        else{
//        //            candidates.insert(candidates.end(), entries.end()-nretrieved, entries.end());
//        //            addi_candidates.insert(addi_candidates.end(), entries.begin(), entries.end()-nretrieved);
//        //        }

//        if(active_hashtables[k])
//            candidates.insert(candidates.end(), entries.begin(), entries.end());
//        else
//            addi_candidates.insert(addi_candidates.end(), entries.begin(), entries.end());
//    }
//}

void MultiIndexHashing::query(const unsigned int *pDesc, std::vector<Entry> &candidates, std::vector<Entry> &addi_candidates, std::vector<int> &nfeatures)
{
    //    std::unique_lock<mutex> lock(mMutexUpdate);

    std::vector<size_t> bucket_idx(hashtable_num);
    //    if(nactive_tables == NULL)
    //        nactive_tables = hashtable_num;
    //    std::vector<unsigned int> active_hashtables(nactive_tables);
    //    std::iota(active_hashtables.begin(), active_hashtables.end(), 0);
    //    cv::Mat mDesc = feature->GetDescriptor();
    //    unsigned int* desc = mDesc.ptr<unsigned int>(); // Weird things happen if use feature->GetDescriptor().ptr<uint32_t>() Why ???
    //    const unsigned int* desc=pDesc;
    // multi-index hash
    hash(pDesc, bucket_idx);
    candidates.clear();
    addi_candidates.clear();

    // duplicate current feature
    //    for(int k=0; k!=hashtable_num; k++){
    for(int k=0; k!=hashtable_num; k++){
        if(nfeatures[k]>0){
            size_t bucket_id = bucket_idx[k];

            // query the features stored in the same entry
            std::vector<Entry> entries;
            //            hashtables[k].getEntries(bucket_id, entries, nfeatures[k] + dynamics);
            mvvBuckets[k][bucket_id].back(entries, nfeatures[k] + dynamics);

            // merge features in all the entries
            //#ifdef ONLINE_TABLE_SELECTION
            //            for(int p=0; p!=entries.size(); p++){
            //                entries[p]->mvbActiveHashTables[k] = true;

            ////                ORB_SLAM::MapPoint *pMP = entries[p];
            ////                if(!pMP->queriedByHashing){
            ////                    pMP->queriedByHashing = true;
            ////                    unsigned int pDesc2 = pMP->GetDescriptor().ptr<unsigned int>(0);

            ////                    if(bits_per_substring == 8)
            ////                    {
            ////                        unsigned char * entry_value1 = (unsigned char *)pDesc;
            ////                        unsigned char * entry_value2 = (unsigned char *)pDesc2;
            ////                        for(int i=0; i< hashtable_num; i++)
            ////                            if(entry_value1[i] == entry_value2[i])
            ////                                pMP->mvbActiveHashTables[i] = true;
            ////                    }
            ////                    else if(bits_per_substring == 16)
            ////                    {
            ////                        unsigned short * entry_value1 = (unsigned short *)pDesc;
            ////                        unsigned short * entry_value2 = (unsigned short *)pDesc2;
            ////                        for(int i=0; i< hashtable_num; i++)
            ////                            if(entry_value1[i] == entry_value2[i])
            ////                                pMP->mvbActiveHashTables[i] = true;
            ////                    }
            ////                    else{
            ////                        cerr<<"ERROR: unsupported substring length!!!"<<endl;
            ////                        exit(-1);
            ////                    }

            ////                }
            //            }
            //#endif

            //            cout << "entries.size() = " << entries.size() << endl;
            if(entries.size() <= NUM_CANDIDATE_FEATURES + dynamics){
                candidates.insert(candidates.end(), entries.begin(), entries.end());
            }
            else{
                //                cout << "candidate size before insertion = " << candidates.size();
                candidates.insert(candidates.end(), entries.end() - NUM_CANDIDATE_FEATURES - dynamics, entries.end());
                //                cout << "candidate size after insertion = " << candidates.size();
                addi_candidates.insert(addi_candidates.end(), entries.begin(), entries.end() - NUM_CANDIDATE_FEATURES - dynamics);
            }
        }
    }
}



//void MultiIndexHashing::printBuckets(const unsigned int *pDesc, unsigned int nactive_tables)
//{
//    std::vector<size_t> bucket_idx(hashtable_num);
//    if(nactive_tables == NULL)
//        nactive_tables = hashtable_num;
//    std::vector<unsigned int> active_hashtables(nactive_tables);
//    std::iota(active_hashtables.begin(), active_hashtables.end(), 0);

//    // multi-index hash
//    hash(pDesc, bucket_idx);

//    // duplicate current feature
//    for(auto hashtable_id : active_hashtables){
//        size_t bucket_id = bucket_idx[hashtable_id];
//        // query the features stored in the same entry
//        std::vector<Entry> entries;
//        hashtables[hashtable_id].getEntries(bucket_id, entries, MAX_BUCKET_SIZE);

//        for(auto e : entries)
//            cout<<e->mnId<<",";
//        cout<<endl;
//    }
//}

unsigned int MultiIndexHashing::getSubstringLen()
{
    return bits_per_substring;
}

int MultiIndexHashing::updateDynamics(double e)
{
    if (abs(e) < MIN_TIME_DIFF)
        return dynamics;

    if (e>0 && dynamics + NUM_CANDIDATE_FEATURES > MIN_BUCKET_SIZE){   // reduce
        dynamics --;
    }

    if (e<0 && dynamics + NUM_CANDIDATE_FEATURES < MAX_BUCKET_SIZE){
        dynamics ++;
    }

    //    cout<<"Dynamics changes to: "<<dynamics<<endl;
    return dynamics;
}

//HashTableSet::HashTableSet(const size_t table_size)
//{
//    // initialize hash tables
//    mvvpBuckets.resize(NUM_TOTAL_HASHTABLES);
//    for(int i=0; i!=NUM_TOTAL_HASHTABLES; ++i){
//        mvvpBuckets[i].resize(table_size);
//    }
//}

//void MultiIndexHashing::printHashTableDistribution(std::string filename)
//{
//    std::ofstream out(filename, std::ios::out);
//    if(out.is_open())
//    {
//        for(size_t i=0; i!=hashtable_num; i++)
//        {
//            HashTable& ht = hashtables[i];
//            for(size_t j=0; j!=bucket_num_per_hashtable; j++)
//            {
//                if(j!=bucket_num_per_hashtable-1)
//                    out << ht.getEntrySize(j) << ",";
//                else
//                    out << ht.getEntrySize(j) << std::endl;
//            }
//        }
//        std::cout<<"Finish saving hashtable distritbuions at "<<filename<<std::endl;
//    }
//    else
//    {
//        std::cerr<<"ERROR: Unable to save hashtable distritbuions at "<<filename<<std::endl;
//    }
//    out.close();
//}


//void MultiIndexHashing::printTableSize(/*const unsigned int tableid*/)
//{
//    std::cout<<"#################################"<<std::endl;
//    std::cout<<"Size of Buckets per HashTable:"<<std::endl;
//    for(auto it=hashtables.begin(); it!= hashtables.end(); it++)
//    {
//        if(it!=hashtables.end()-1)
//            std::cout<<it->getBucketSize()<<",";
//        else
//            std::cout<<it->getBucketSize()<<std::endl;
//        //    std::cout<<std::endl;
//    }

//    //    std::cout<<std::endl;
//    std::cout<<"Size of Sum Entries per HashTable:"<<std::endl;
//    for(auto it=hashtables.begin(); it!= hashtables.end(); it++)
//    {
//        if(it!=hashtables.end()-1)
//            std::cout<<it->getHashTableSize()<<",";
//        else
//            std::cout<<it->getHashTableSize()<<std::endl;
//        //    std::cout<<std::endl;
//    }
//    std::cout<<"#################################"<<std::endl;
//}

//size_t MultiIndexHashing::getTableSize(const unsigned int tableid)
//{
//    return hashtables[tableid].getBucketSize();
//}

//unsigned int MultiIndexHashing::getHashTableSize()
//{
//    return hashtable_num;
//}

//size_t MultiIndexHashing::getBucketNum()
//{
//    return bucket_num_per_hashtable;
//}

////template<typename FeatureType>
//void MultiIndexHashing::insertNquery(const Feature feature, std::vector<unsigned int> &feature_list)
//{
//    feature_indicator f;
//    f.feature_index = feature.feature_index;
//    f.image_index = feature.image_index;
////    f.global_idx = feature.global_index;

//    std::vector<unsigned long> hash_entry_index(hashtable_num);

//    unsigned int* desc = (unsigned int*)feature.desc;

//    is_empty_hashtable = false;

//    // multi-index hash
//    multi_index_hash(hash_entry_index, desc);

//    candidates_list.clear();
//    // duplicate current feature
//    for(int hashtable_index = 0; hashtable_index < hashtable_num; hashtable_index++){
//        unsigned long entry_index = hash_entry_index[hashtable_index];

//        // query the features stored in the same entry
//        entry entry_items = features_buffer[hashtable_index][entry_index];

//        // merge features in all the entries
//        for(auto it=entry_items.begin(); it!=entry_items.end(); it++)
//            candidates_list.insert(*it);

//        if(features_buffer[hashtable_index][entry_index].size() == 0 ||
//                (features_buffer[hashtable_index][entry_index].size() < MIH_MAX_NUM_PER_ENTRY &&
//                 (features_buffer[hashtable_index][entry_index]).back().image_index != f.image_index)){

//            features_buffer[hashtable_index][entry_index].push_back(f);
//        }
//    }

//    return candidates_list.size();
//}

}
