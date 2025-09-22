# Suspension Kinematics Analysis

A collection of tools for analyzing vehicle suspension kinematics, including both MATLAB and Python implementations for double-wishbone and trailing arm suspension systems.

**Author:** Jeremy Kimball

## Overview

This repository contains kinematic analysis tools for two common suspension types:

- **Double-Wishbone Suspension** - Analyzes linkage positions based on lower control arm angle
  - Geometry reference: [Project Chrono ChDoubleWishbone](https://api.projectchrono.org/wheeled_suspension.html)
- **Trailing Arm Suspension** - Analyzes linkage positions based on trailing arm angle
  - Geometry reference: [Project Chrono ChThreeLinkIRS](https://api.projectchrono.org/wheeled_suspension.html) 

Given a suspension angle as input, this code calculates 3D positions of suspension components and provides interactive visualizations of the suspension geometry.

This code has been validated against the Project Chrono Polaris vehicle implementation to ensure accuracy of the kinematic calculations.

## Dependencies

### MATLAB
- MATLAB R2018b or later
- No additional toolboxes required

### Python
- Python 3.7+
- NumPy
- Matplotlib
- SciPy
- PyQt5 (for interactive plots)

Install Python dependencies:
```bash
cd python/
pip install -r requirements.txt
```

## Usage

### MATLAB
```matlab
cd matlab/
run_kinematic_test_DWB  % Double-wishbone analysis
run_kinematic_test_TA   % Trailing arm analysis
```

### Python
```bash
cd python/
python run_kinematic_test_dwb.py  # Double-wishbone analysis
python run_kinematic_test_ta.py   # Trailing arm analysis
```

## Coordinate System

All calculations use a vehicle-centric coordinate system:
- **X-axis**: Forward (vehicle longitudinal)
- **Y-axis**: Left (vehicle lateral)
- **Z-axis**: Up (vehicle vertical)

Units are in meters.

## Features

- 3D kinematic analysis of suspension linkages
- Interactive 3D visualizations
- Calculation of suspension component positions
- Support for custom suspension geometries
- Both analytical (double-wishbone) and optimization-based (trailing arm) solutions

## License

This project is developed for educational and research purposes.
