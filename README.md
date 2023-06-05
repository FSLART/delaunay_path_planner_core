# Delaunay Path Planner Core

This node is responsible for planning a path between the cones.
This is the core library, so the only thing you can execute are the tests.

## Requirements
- CGAL
- [lart_common](https://github.com/FSIPLEIRIA/lart_common) - installation instructions on the repository

## Setup

### Bare metal

- Create the directory ```lib``` and ```cd lib```.
- Clone gtest ```git clone https://github.com/google/googletest.git```.
- Head back to this directory ```cd..```.
- Create a directory in this location, for example ```mkdir build``` and ```cd build```.
- Run the command ```cmake ..```.
- Run ```make```.
- Run ```sudo make install```.

If you wish to run the tests, go to the ```build``` directory and run ```./test```.

### Docker

- Run the command ```docker build -t delaunay_path_planner .```.
- Run the command ```docker run delaunay_path_planner```.

The tests will be run inside the container.
