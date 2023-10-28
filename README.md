# Eigen_UKF

A C++ implementation of the Unscented Kalman Filter (UKF) using Eigen. For the implementation I took inspiration from https://github.com/CoffeeKumazaki/kalman_filters.

The UKF uses a deterministic sampling technique known as the unscented transformation to calculate statistics around the mean. This technique does not require differentiability of the models.

## Installation

### Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) library.

### Tested only on Ubuntu 20.04 LTS

Clone the repo on your PC. Then, enter the folder where you downloaded the repo, open a terminal and run:

```
mkdir build && cd build
cmake ../
make
```

The executable of the c++ code will be available in the `build/` folder.

