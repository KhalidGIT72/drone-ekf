# 2D Drone State Estimation — Kalman Filter in C++

Kalman Filter for 2D drone position and velocity estimation. Algorithm prototyped in MATLAB, implemented in C++ using Eigen, validated against simulated GPS data.

## System Model
- State: `[x, y, vx, vy]`
- Measurements: GPS position only — velocity estimated by the filter
- Process noise: random accelerations (σ = 0.2 m/s²)
- GPS noise: σ = 2.0 m

## Dependencies
- Eigen 5.0.0
- CMake 3.10+
- MATLAB
- C++17, MSVC 2019

## Build
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Run
1. Run `matlab/generate_simulation.m`
2. From `build/`: `Debug\drone_ekf.exe`
3. Run `matlab/validate_ekf.m`

## Results
[Position](results/ekf_position_time.png)
[Trajectory](results/ekf_trajectory.png)