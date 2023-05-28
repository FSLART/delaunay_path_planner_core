FROM ubuntu:22.04

# copy source
COPY . /usr/src/delaunay_path_planner_core
WORKDIR /usr/src/delaunay_path_planner_core

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt install git build-essential cmake libcgal-dev -y

# clone gtest
RUN mkdir lib
WORKDIR /usr/src/delaunay_path_planner_core/lib
RUN git clone https://github.com/google/googletest.git /usr/src/delaunay_path_planner_core/lib/googletest

# build
RUN mkdir build
WORKDIR /usr/src/delaunay_path_planner_core/build
RUN cmake ..
RUN make -j4

# run all tests
ENTRYPOINT ["./test"]
