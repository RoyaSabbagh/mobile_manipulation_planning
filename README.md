# mobile_manipulation_planning

This is a ROS package that represents a MPC and optimization-based motion and manipulation planning for a mobile robot to deliver legged objects.

It is authored by [Roya Sabbagh Novin](https://sites.google.com/view/roya-sn), [Amir Yazdani](https://amir-yazdani.github.io/), [Andrew Merryweather](https://mech.utah.edu/faculty/andrew-merryweather/), and [Tucker Hermans](https://robot-learning.cs.utah.edu/thermans) from Utah Robotics Center, University of Utah.
        
        
   [![Sammary](http://img.youtube.com/vi/mw9qBr66bVQ/0.jpg)](http://www.youtube.com/watch?v=mw9qBr66bVQ "Summary")



### Features
- **Functionality**:
  - **Motion planning for a low-cost mobile robot** ([iRobotCreate 2](https://store.irobot.com/default/create-programmable-programmable-robot-irobot-create-2/RC65099.html?gclid=Cj0KCQiAuefvBRDXARIsAFEOQ9E11JETRssFIy08ObY0O_aYGBX1uW5f1_GBXpzAAH5eRcnfsAQmAWwaAkt1EALw_wcB&gclsrc=aw.ds)) 
    - Minimum-path motion planning
    - Convex optimization formulation
    - MPC-based re-planning
    - Pose feedback from motion capture
  - **Hybrid manipulation planning to move legged objects to a target**
    - Using the learned the dynamics of the legged objects ([Github repository for dynamics learning](https://github.com/RoyaSabbagh/dynamics_model_learning))
    - Minimum-path manipulation planning for both the robot and the object
    - Convex optimization formulation
    - MPC-based re-planning
    - Planning for both position and orientation of the object
    - Push and pull planning
    - Repositioning of the robot wrt the object mainaining the grasping leg
    - Leg change if required
    - Grasping and regrasping the object legs during repositioning and leg change
    - Pose feedback from motion capture
  - **Visualization**
    - Rviz visualization for simulation results
    - Including urdf for common objects in a hospital room
    - calibration and overlaying the results on the video for physical experiments
- **Input**: Robot intial position, object type, object initial position and orientation, object goal position and orientation, obstacle position and orientation, layot of the environment
- **Output**: Planned path for robot and object, executed path for robot and object
- **Operating System**: Ubuntu (16.04), ROS kinetic
- **Dataset**: [Dataset for dynamics learning](https://github.com/RoyaSabbagh/dynamics_model_learning/tree/master/Data)

### Results
   - **Simulation results** 
      - **Manipulating a walker**
      
      [![Simulation-Walker](http://img.youtube.com/vi/_du8ay1qxXU/0.jpg)](http://www.youtube.com/watch?v=_du8ay1qxXU "Simulation-Walker")
      
      - **Manipulating a chair (1)**
      
      [![Simulation-chair (1)](http://img.youtube.com/vi/stxh9080M_o/0.jpg)](http://www.youtube.com/watch?v=stxh9080M_o "Simulation-chair (1)")
   
      - **Manipulating a chair (2)**
      
      [![Simulation-chair (2)](http://img.youtube.com/vi/WK6hDusN3RQ/0.jpg)](http://www.youtube.com/watch?v=WK6hDusN3RQ "Simulation-chair (2)")
      
      - **Manipulating a cart**
      
      [![Simulation-cart](http://img.youtube.com/vi/dO5V90yQYqg/0.jpg)](http://www.youtube.com/watch?v=dO5V90yQYqg "Simulation-cart")
    
   - **Physical experiment results** 
      - **Manipulating a walker**     
      
      [![physical experiment-walker](http://img.youtube.com/vi/RrlDCtD12uI/0.jpg)](http://www.youtube.com/watch?v=RrlDCtD12uI "physical experiment-walker")

 
### Dependencies ###

  - **Gurobi optimization package**
    Install Gurobi for ubuntu using this [link](https://www.gurobi.com/documentation/5.6/quickstart/installation_linux.html)

### Installation ###
1. Make sure you have ROS-kinetic installed and your catkin workspace is generated
2. Clone the package in your catkin workspace
3. Make/build your catkin workspace
4. Enjoy!


### How to set up and run ###
1. Set a simulation example in "Examples_sim.py" or choose one from the available ones
**Hint**: Running real experiments requiers the lab setup as described in the paper
2. Assign the example number in "main_simulation.py"
3. Run "launch_simulation.launch"
4. Run "rviz.launch" for visualization in rviz



