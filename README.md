This is the second version of RGBD SLAM tutor. Please visit [my blogs](http://www.cnblogs.com/gaoxiang12) for details: 

---
# Installization
1. Dependencies: OpenCV 2.4.x , [PCL 1.7](http://pointclouds.org/), Eigen. Use commands below if you are using Ubuntu: 

    sudo apt-get install libopencv-dev libeigen3-dev

2. Compile third-party libs, including [DBoW2](https://github.com/raulmur/ORB_SLAM2) (for loop closure), a modified version of [g2o](https://github.com/RainerKuemmerle/g2o) (for solving pnp), and the OrbExtractor from [orb-slam2] (https://github.com/raulmur/ORB_SLAM2).

  They are all cmake projects, so just go into the directory and type:
```
    mkdir build
    cmake ..
    make -j2
```
  For g2o you need to type *make install* to install it into /usr/local/ otherwise the FindG2O.cmake will not work.


3. Compile this project:

```
    mkdir build
    cmake ..
    make -j2
```

You will find some experiment binaries in *bin/* like bin/exp_mapping, they are experiments described in the blog. Please download the dataset and edit the parameter file before doing experiments. 

Many thanks to the excellent works of g2o and orb-slam!

