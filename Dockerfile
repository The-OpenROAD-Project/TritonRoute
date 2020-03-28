FROM centos:centos6 AS builder

# install gcc 8
RUN yum -y install centos-release-scl && \
    yum -y install devtoolset-8 devtoolset-8-libatomic-devel
ENV CC=/opt/rh/devtoolset-8/root/usr/bin/gcc \
    CPP=/opt/rh/devtoolset-8/root/usr/bin/cpp \
    CXX=/opt/rh/devtoolset-8/root/usr/bin/g++ \
    PATH=/opt/rh/devtoolset-8/root/usr/bin:$PATH \
    LD_LIBRARY_PATH=/opt/rh/devtoolset-8/root/usr/lib64:/opt/rh/devtoolset-8/root/usr/lib:/opt/rh/devtoolset-8/root/usr/lib64/dyninst:/opt/rh/devtoolset-8/root/usr/lib/dyninst:/opt/rh/devtoolset-8/root/usr/lib64:/opt/rh/devtoolset-8/root/usr/lib:$LD_LIBRARY_PATH

RUN yum install -y wget git pcre-devel tcl-devel tk-devel bison flex \
                   python-devel libxml2-devel libxslt-devel zlib-static glibc-static

RUN wget http://prdownloads.sourceforge.net/swig/swig-4.0.0.tar.gz && \
    tar -xf swig-4.0.0.tar.gz && \
    cd swig-4.0.0 && \
    ./configure && \
    make && \
    make install

# installing cmake for build dependency
RUN wget https://cmake.org/files/v3.14/cmake-3.14.0-Linux-x86_64.sh && \
    chmod +x cmake-3.14.0-Linux-x86_64.sh  && \
    ./cmake-3.14.0-Linux-x86_64.sh --skip-license --prefix=/usr/local

# installing boost for build dependency
RUN wget https://dl.bintray.com/boostorg/release/1.68.0/source/boost_1_68_0.tar.bz2 && \
    tar -xf boost_1_68_0.tar.bz2 && \
    cd boost_1_68_0 && \
    ./bootstrap.sh && \
    ./b2 install

COPY . /TritonRoute
RUN mkdir TritonRoute/build
WORKDIR /TritonRoute/build
RUN cmake ..
RUN make


FROM centos:centos6 AS runner
RUN yum update -y && yum install -y tcl-devel
COPY --from=builder /TritonRoute/build/TritonRoute /build/TritonRoute

RUN useradd -ms /bin/bash openroad
USER openroad
WORKDIR /home/openroad
