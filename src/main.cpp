/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <chrono>
#include "FlexRoute.h"

using namespace std;
using namespace fr;

int readParams(const string &fileName) {
  int readParamCnt = 0;
  fstream fin(fileName.c_str());
  string line;
  if (fin.is_open()){
    while (fin.good()){
      getline(fin, line);
      if (line[0] != '#'){
        char delimiter=':';
        int pos = line.find(delimiter);
        string field = line.substr(0, pos);
        string value = line.substr(pos + 1);
        stringstream ss(value);
        if (field == "lef")           { LEF_FILE = value; ++readParamCnt;}
        else if (field == "def")      { DEF_FILE = value; ++readParamCnt;}
        else if (field == "guide")    { GUIDE_FILE = value; ++readParamCnt;}
        else if (field == "outputTA") { OUTTA_FILE = value; ++readParamCnt;}
        else if (field == "output")   { OUT_FILE = value; ++readParamCnt;}
        else if (field == "outputguide") { OUTGUIDE_FILE = value; ++readParamCnt;}
        else if (field == "outputMaze") { OUT_MAZE_FILE = value; ++readParamCnt;}
        else if (field == "outputDRC") { DRC_RPT_FILE = value; ++readParamCnt;}
        else if (field == "threads")  { MAX_THREADS = atoi(value.c_str()); ++readParamCnt;}
        else if (field == "verbose")    VERBOSE = atoi(value.c_str());
        else if (field == "dbProcessNode") { DBPROCESSNODE = value; ++readParamCnt;}
      }
    }
    fin.close();
  }
  if (readParamCnt < 5) {
    return 2;
  } else {
    return 0;
  }
}

int main(int argc, char** argv) {

  //cout <<CPX_MIN <<endl;
  //return 0;
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //double startTime = omp_get_wtime();

  //std::ios::sync_with_stdio(false);
  cout <<"TritonRoute Version 0.0.6.0" <<endl;
  cout <<"Developed by Lutong Wang and Bangqi Xu\n"
       <<"\n"
       <<"Copyright (c) 2019, The Regents of the University of California\n"
       <<"All rights reserved.\n"
       <<"\n"
       <<"Redistribution and use in source and binary forms, with or without\n"
       <<"modification, are permitted provided that the following conditions are met:\n"
       <<"    * Redistributions of source code must retain the above copyright\n"
       <<"      notice, this list of conditions and the following disclaimer.\n"
       <<"    * Redistributions in binary form must reproduce the above copyright\n"
       <<"      notice, this list of conditions and the following disclaimer in the\n"
       <<"      documentation and/or other materials provided with the distribution.\n"
       <<"    * Neither the name of the University nor the\n"
       <<"      names of its contributors may be used to endorse or promote products\n"
       <<"      derived from this software without specific prior written permission.\n"
       <<"\n"
       <<"THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND\n"
       <<"ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED\n"
       <<"WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE\n"
       <<"DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY\n"
       <<"DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES\n"
       <<"(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;\n"
       <<"LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND\n"
       <<"ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
       <<"(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS\n"
       <<"SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
       <<"\n";
  if (argc == 1) {
    cout <<"Error: usage ./TritonRoute -lef <LEF_FILE> -def <DEF_FILE> -guide <GUIDE_FILE> -output <OUTPUT_DEF>" <<endl;
    return 2;
  }

  if (argc == 2) {
    int readSuccess = readParams(string(argv[1]));
    if (readSuccess) {
      cout <<"Error reading param file!!!" <<endl;
      return 2;
    }
  } else {
    argv++;
    argc--;
    while (argc--) {
      if (strcmp(*argv, "-lef") == 0) {
        argv++;
        argc--;
        LEF_FILE = *argv;
        //cout <<"lef: " <<LEF_FILE <<endl;
      } else if (strcmp(*argv, "-def") == 0) {
        argv++;
        argc--;
        DEF_FILE = *argv;
        //cout <<"def: " <<DEF_FILE <<endl;
      } else if (strcmp(*argv, "-guide") == 0) {
        argv++;
        argc--;
        GUIDE_FILE = *argv;
        //cout <<"guide: " <<GUIDE_FILE <<endl;
      } else if (strcmp(*argv, "-threads") == 0) {
        argv++;
        argc--;
        sscanf(*argv, "%d", &MAX_THREADS);
        //cout <<"thread: " <<MAX_THREADS <<endl;
      } else if (strcmp(*argv, "-output") == 0) {
        argv++;
        argc--;
        OUT_FILE = *argv;
        //cout <<"output: " <<OUT_FILE <<endl;
      } else if (strcmp(*argv, "-verbose") == 0) {
        argv++;
        argc--;
        VERBOSE = atoi(*argv);
        //cout <<"output: " <<OUT_FILE <<endl;
      } else {
        cout <<"ERROR: Illegal command line option: " <<*argv <<endl;
        return 2;
      }
      argv++;
    }
  }
  
  FlexRoute router;
  router.main();
  
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  if (VERBOSE > 0) {
    cout <<endl <<"Runtime taken (hrt): " << time_span.count()    <<endl;
  }
  return 0;
}
