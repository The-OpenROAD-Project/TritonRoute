# TritonRoute
**TritonRoute** is an open source detailed router for modern industrial 
designs. The router consists of several main building blocks, including 
track assignment, initial detailed routing, search and repair, and a DRC 
engine.  The initial development of the 
[router](https://vlsicad.ucsd.edu/Publications/Conferences/363/c363.pdf) 
is inspired by the 
[ISPD-2018 initial detailed routing contest](http://www.ispd.cc/contests/18/). 
However, the current framework differs and is built from scratch, aiming 
for an industrial-oriented scalable and flexible flow. Currently, the router 
only has a limited support of block-level designs with standard cells and 
macros for CLN65LP.

TritonRoute was developed by graduate students Lutong Wang and Bangqi Xu from 
UC San Diego, and serves as the detailed router in the 
[OpenROAD](https://theopenroadproject.org/) project. 

TritonRoute provides industry standard LEF/DEF interface with 
support of [ISPD-2018](http://www.ispd.cc/contests/18/) and 
[ISPD-2019](http://www.ispd.cc/contests/19/) contest-compatible route guide 
format.

## Installation ##
TritonRoute is tested in 64-bit CentOS 6/7 environments with the following
prerequisites:
* A compatible C++ compiler supporting C++17 (GCC 7 and above)
* Boost >= 1.68.0
* Bison >= 3.0.4
* zlib >= 1.2.7
* CMake >= 3.1

To install TritonRoute:
```
$ git clone https://github.com/The-OpenROAD-Project/TritonRoute.git
$ cd TritonRoute 
$ mkdir build
$ cd build
$ cmake -DBOOST_ROOT=<BOOST_ROOT> ../
$ make
```
   
To run TritonRoute: 
```
$ ./TritonRoute -lef <LEF_FILE> -def <DEF_FILE> -guide <GUIDE_FILE> -output <OUTPUT_DEF>
```

## Supported Technologies ##
* CLN65LP (with limited selection of standard cells, macros and floorplans)
* (TritonRoute is under a major improvement plan for a more robust and stable 
flow, with support of more standard cells, macros and floorplans in CLN65LP 
and other technology nodes. The next version is expected to deliver support 
of publicly accessible ISPD-2018 and ISPD-2019 testcases.)


## License ##
* [BSD 3-clause License](LICENSE) 

