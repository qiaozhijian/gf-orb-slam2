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

#include <time.h>

#include "ORBVocabulary.h"
using namespace std;

bool load_as_text(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  bool res = voc->loadFromTextFile(infile);
  printf("Loading fom text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return res;
}

void load_as_xml(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->load(infile);
  printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void load_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->save(outfile);
  printf("Saving as xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToTextFile(outfile);
  printf("Saving as text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToBinaryFile(outfile);
  printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}


int main(int argc, char **argv) {
  cout << "BoW load/save benchmark" << endl;
  ORB_SLAM2::ORBVocabulary* voc = new ORB_SLAM2::ORBVocabulary();

  load_as_text(voc, "../ORB_Data/ORBvoc.txt");
  save_as_binary(voc, "../ORB_Data/ORBvoc.bin");

  return 0;
}
