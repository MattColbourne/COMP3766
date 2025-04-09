**ROS-Based Numerical Inverse Kinematics for a PUMA 6R Robot Using Gradient Descent Optimization**

Name: Matthew Colbourne     	Student ID: 202132726 <br>
Name: Shuvojit Nandi            Student ID: 202169637

-------------------------------------

Devcontainer instructions:

Install VSCode, devcontainer extension and Docker.
Clone the repository:

    $ git clone https://github.com/MattColbourne/COMP3766.git

On Windows:

    $ git clone -c core.autocrlf=false https://github.com/MattColbourne/COMP3766.git

Open the folder in VSCode.
Select the option "Reopen in container" from the green square at the bottom or from the menu.  
Wait it finishing loading the folder in Docker (it can take a while).
Open a terminal. 

After this you should be able to open the browser on `localhost:6080` and see the GUI (RViz and Gazebo).  
It the GUI asks for a password: `vscode`.  

--------------------------------------
If you get any error saying '/python3/r', then you have to discard the changes or stash the changes. 
---------------------------------------

Step 1:
make sure that the terminal is currently running under COMP3766.

Step 2:
In a **new terminal**, run: <br>
    `catkin_make` <br>
    `source devel/setup.bash` <br>
    `roslaunch lab3 lab3.launch`<br>

Step 3:
Open : http://localhost:6080/ <br>
You will see that the robot will look "broken"  but it is just because the robot does not have the joints published yet.

Step 4:
In a **new terminal**, run: <br>
    `source devel/setup.bash` <br>
    `roslaunch lab3 numerical_IK.py`<br>

you should get : **Waiting for /goal_pose** <br>

Step 5:
In a **new terminal**, run: <br>
    `source devel/setup.bash` <br>
    `roslaunch lab3 goal_pose_node.py`<br>

you should see the completed robot. If you want to change the goal position then change the code in **goal_pose_node.py** and do step 4 and step 5 again.



