# fourleg_gazebo

Thing to prepare prior runing the script:
** Gurobi license is require for runninng this simulation, may refer to:
https://www.gurobi.com/downloads/free-academic-license/#show_instructions

This simulation package is the walking robot control with the optimization of  the center of mass of robot in performing the stable walk
The optimization policy is constraint with the zero moment point theory, smoothing the continuous trajectory, and the modal of optimization used is the mixed integer quadratic programming.

The reference of this algorithm is:
M. S. Ahn, H. Chae, and D. W. Hong, Unknown Terrain Locomotion for Quadrupeds based on Visual Feedback and Mixed-Integer Convex Optimization, In: IEEE Int. Conf. Intell. Robots Syst(IROS) pp 3791-3798 (2018)


************************************************************************************************************************************
Manual in launching robot simulation platoform and running walking simulation:


1. Launch the robot into gazebo platform:

	roslaunch fourleg_gazebo hello.launch

OPEN ANOTHER TAB
chanege to script directory : cd ~/catkin_ws/src/fourleg_gazebo/script
2.open the virtual environment for running the optimization in anaconda
	conda config --set auto_activate_base True     ←{set it False to deactivate it}
	source ~/.bashrc

3.Run the simulation in ros
	 conda activate ros_env

	##for running with QP optimizer
	./JP_COM_QP_OMNI_DIR_FV3_v2.py   

	##for running with trained joint
	./Joint_publisher_with_trained_joint.py
