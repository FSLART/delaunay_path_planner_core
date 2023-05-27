FROM ubuntu:22.04

# copy source
COPY . /usr/src/delaunay_path_planner_core
WORKDIR /usr/src/delaunay_path_planner_core

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt install build-essential cmake libcgal-dev

# build
RUN mkdir build
WORKDIR /usr/src/delaunay_path_planner_core/build
RUN cmake ..
RUN make -j4

# run all tests
ENTRYPOINT ["./test"]