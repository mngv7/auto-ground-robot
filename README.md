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
5. Run startupGround.m once more to initialise the simulation 