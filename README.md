# Auto Ground Robot EGH446
## Depedencies
- MATLAB 2025a
- Simulink
- Stateflow
- Robotics System Toolbox
- Navigation Toolbox
- Control System Toolbox + Model Predictive Control Toolbox (optional)
- Reinforcement Learning Toolbox + Deep Learning Toolbox (optional)
- Simscape + Simscape Multibody (optional)

## Project Structure

- `functions/`: Contains custom MATLAB functions used in the project.
- `toolboxes/`: Contains robotics toolboxes and additional libraries.

## How to Use
1. Clone the repository to your local machine.
2. Open MATLAB and navigate to the project directory.
3. Ensure all dependencies are installed.
4. Run startupGround.m to initalize the project environment, it will have an error as the robot model is not yet loaded. (This is expected)
5. Click run in the simulink environment to load the robot model. It will have an error as workspace variables have not been loaded (This is also expected).
6. Run startupGround.m once more to initialise the simulation. There will now be a delay while the path planning algorithm runs, map is loaded, and simulink model compiles with the workspace variables. After this, a new figure will appear and the simulation will run.

## Figures and Data Outputs
Several figures are produced when running this simulation in addition to the Robot Visualization (simulation).
The first is a figure plotting the Cross-Track Error over the course of the run. The second will be the environment with the closest distances that the robot achieved to each waypoint (NB: these may appear out of order due to the processing needed for calculating cross-track error).
