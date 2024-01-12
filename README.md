# Eigen_UKF

[![CI_UKF](https://github.com/gabrielenava/Eigen_UKF/actions/workflows/ci_ukf.yml/badge.svg)](https://github.com/gabrielenava/Eigen_UKF/actions/workflows/ci_ukf.yml)

A C++ implementation of the Unscented Kalman Filter (UKF) using Eigen. The UKF uses a deterministic sampling technique known as the unscented transformation to calculate statistics around the mean. This technique does not require differentiability of the models. 

See also https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf for more details.

For the implementation I took inspiration from https://github.com/CoffeeKumazaki/kalman_filters.

## Installation and usage

**Tested only on Ubuntu 20.04 LTS**

### Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) library.

### Compilation

Clone the repo on your PC. Then, enter the folder where you downloaded the repo, open a terminal and run:

```
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="/path/to/desired/install/dir"
make install
```

The UKF library will be available in the `install/lib` folder. The library is composed of two classes:

- [SystemModel](lib/src/SystemModel.cpp): template class used to construct the state and observation models. Create a child class from this parent class, then edit it to add your models.
- [UnscentedKF](lib/src/UnscentedKF.cpp): the class implementing the UKF algorithm.

### Example

An example of usage of the library is available in the [example](example) folder. The filter is used to estimate the thrust provided by a jet engine and its rate of change, given the measured thrust data.

### Test

A simple test to verify library integrity is provided in the [test](test) folder. The test uses [Catch2](https://github.com/catchorg/Catch2.git) library and can be run with the `ctest` command from the `build` directory.

## Maintainer

Gabriele Nava, @gabrielenava
