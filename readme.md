# Reinforcement Learning with Hand, Arm, Ball, and Target

![91tq56](https://github.com/user-attachments/assets/6d6aa8f1-bb2f-4b43-8bf5-729e08e61165)
![91tqee](https://github.com/user-attachments/assets/5594256f-c453-4f52-ba7a-844ef9eeaa43)
![91tr34](https://github.com/user-attachments/assets/8a27663a-56ef-4813-935b-09270f40613b)

## Youtube video

[https://youtu.be/dqIwWVMw7HM](https://www.youtube.com/watch?v=_yNpbxOqKTU)

## Installation / Running

In order to run this scene, load it in the Unity editor (2021.3.9f).

Most of the relevant settings for demo-ing are under "Inference Settings" on the `Singletons > ConfigManager` object. And the Agent Neural Network can be changed by altering the model weights under "ML_Agent" or "ML_Agent_AddTouchData" on the `MLAgentTemplates` scene object.

Once the scene is playing, you can use the keyboard to move around the environment, use WASD for 2d movement, Space and Ctrl for up/down movement, and Q for locking and unlocking the camera on the projectile.

## Overview 
I adapted Ali Bharwan's [Drecon Unity implementation]([https://github.com/orangeduck/Motion-Matching](https://github.com/AliBharwani/Drecon)) [[1]](#citation1) for this project.
 
Most of the logic lives in `MLAgent.cs`

The script execution order is set in the Player Settings. The anatomy of a frame:

(1) `MLAgent.cs` updates observation state variables that are dependent on the kinematic character. It either requests an action or applies the actions from the last network output. 

(2) `AcademyManager.cs` runs one step. If `MLAgent.cs` requested a decision, it will execute the `CollectObservations()` and `OnActionReceived()`  functions of `MLAgent.cs`

(3) `PhysicsManger.cs` steps the simulation by the fixed timestep. This has to run after the PD motor targets are updated by the network in (3) or (4) 

(4) `MultiMLAgentsDirector.cs` executes. This calls the `LateFixedUpdate()` method on `MLAgent.cs`, which calculates the rewards and updates data dependent on the simulated model (center of mass, etc) that changed after the physics step. 

### ConfigManager
The ConfigManager object controls all of the relevant settings. It also has utility functions for writing and loading to JSON. Depending on the combination of settings, the number of inputs or outputs to the network can change, and `MultiMLAgentsDirector` will instantiate the proper `MLAgent` when the scene is started. 

## Training 

Once you have your ConfigManager setup with the settings you like, you can use the `mlagents-learn` command to train.

By default, `mlagents` does not accept a "init_near_zero" argument in the config. In order to use this feature, you must manually alter your installation of the package. It also restricts the number of available CPUs to 4, and this is a very heavily CPU bottlenecked training, so it is suggested to modify that as well.

The `mlagents-learn` command will, by default, run the simulation at 20x speed. This makes the simulation very unstable, and it's more efficient to run many parallel instances rather than try to run one simulation at that speed. In order to turn this off, be sure to include the args `--time-scale 1 --capture-frame-rate 0`

What I found was the most stable, efficient running setup was to have 2 agents in each build (so make sure the MultiMLAgentsDirector has '2' set for "Num Agents") and run 10-15 no graphics instances, depending on your CPU speed and GPU memory. My final training command would look something like: 
`mlagents-learn Assets\config\Hand.yaml --env="Builds\HandTarget_19_rebalanceNoTarg.exe" --no-graphics --num-env 10 --time-scale 1 --capture-frame-rate 0 --run {run_id}`

Be sure to right click on the ConfigManager in the editor and "Write out current config to config name" in order to save a usable unityconfig to the folder for your training run.

## Future Work

Currently, the model is still unable to hit the target consistently, and doesn't converge to optimal performance on training runs. This could be solved with training with different hyperparameters or environmental parameters.

<a id="citation1"></a>[1] https://github.com/AliBharwani/Drecon
