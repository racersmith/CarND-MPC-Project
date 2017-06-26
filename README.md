# Model Predictive Control
Project for Udacity's Self-Driving Car Engineer Nanodegree Program.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


#  Rubric
## Compilation
Using the basic build instructions above the code compiles and runs.
```bash
josh@T7500:~/CarND-MPC-Project$ mkdir build && cd build
josh@T7500:~/CarND-MPC-Project/build$ cmake .. && make
-- The C compiler identification is GNU 6.3.0
-- The CXX compiler identification is GNU 6.3.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: /CarND-MPC-Project/build
Scanning dependencies of target mpc
[ 33%] Building CXX object CMakeFiles/mpc.dir/src/MPC.cpp.o
[ 66%] Building CXX object CMakeFiles/mpc.dir/src/main.cpp.o
[100%] Linking CXX executable mpc
[100%] Built target mpc
josh@T7500:~/CarND-MPC-Project/build$ 
josh@T7500:~/CarND-MPC-Project/build$ ./mpc
Listening to port 4567
```

## Implementation
### Model
This MPC uses a kinematic model which takes some tire slip angle into consideration. The model is the familiar bicycle model except for yaw which now accounts for some static normal tire load contributing to yaw rate. Put in other words, there is now a car specific gain/attenuation Lf associated with steering angle that is proportional to the position of the center of gravity relative to the steering wheels. The global kinematic model used is found on lines 119-122 in mpc.cpp. To simplify the constraints of the model, the equations are rearranged to equal zero.
```
      0 = x1 - (x0 + v0 * cos(psi0) * dt);
      0 = y1 - (y0 + v0 * sin(psi0) * dt);
      0 = psi1 - (psi0 + v0 * delta0/Lf * dt);
      0 = v1 - (v0 + a0 * dt);
```

Cross track error, CTE, and heading error psides are evaluated by using the path fit polynomial and it's derivative evaluated at the time step's x position. This can be found on lines 128-137 of mpc.cpp.
### Hyperparameters
There are two MPC hyperparameters that are tuned, dt and N, lines 8-12 in mpc.cpp. dt adjusts the time step of the prediction forecast and N sets the number of time steps the MPC forecasts. These parameters were adjusted empirically. The initial values tried were a 0.1s time step with a forecast of 25. This resulted in good results for slower speeds. As the speeds capable increased, the forecast was too far in the future to be reliable and was likely just wasting processing time. A smaller dt was selected and the number of time steps was adjusted to forecast out to the distance of the sensor measurements, which in this simulation was a set of waypoints around the car's position. The final dt selected was 0.05 seconds with a forecast N of 20. The reduction of waypoints to process also decreased computation time and improved simulation behavior. This is likely just a function of my hardware.

### Polynomial Fitting
Before fitting a polynomial to the waypoints the car's state is first updated to handle latency of the system. The waypoints are then transformed to the car's coordinate system, and a third order polynomial is fit.  

### Model Predictive Control with Latency
To simulate a real car where there is latency between an actuation command and the physical actuation, 100ms latency was added to the simulation on line 210 of main.cpp. To account for this latency the car's state was estimated after the latency time before being evaluated by the MPC. This resulted in the controller providing actuation command for a future time aligned with the latency. This was done on lines 109-116 of main.cpp. The same kinematic model was used for this prediction as is used in the MPC.

## Simulation
The car can complete the first three laps in the simulator test track in an average lap time of 30.54 seconds. If the simulator was able to provide more points forward of the car, it is likely possible to achieve much lower lap times. However, the car is traveling fast enough that about half the waypoints are lagging behind the car's position such that only 3-4 waypoints are forward of the car.