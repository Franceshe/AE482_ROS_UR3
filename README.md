# Step-by-step guide on how to reproduce the results from Final Project Checkpoints
Quick reference:  
[Checkpoint1](#Checkpoint1)  
[Checkpoint2](#Checkpoint2)  
[Checkpoint3](#Checkpoint3)  
[Checkpoint4](#Checkpoint4)  
[Checkpoint5](#Checkpoint5)  
[Checkpoint6](#Checkpoint6)  
[Checkpoint7](#Checkpoint7)  
[Checkpoint8](#Checkpoint8)  
[Checkpoint9](#Checkpoint9)  

<a name="Checkpoint1"></a>
## Checkpoint 1 (due 3/5/18) [link to video](https://youtu.be/TBQfd0o0EM0):
### Installing the simulator for Linux (Mint)
Download V-REP PRO EDU from the Coppelia Robotics [website](http://www.coppeliarobotics.com).  
Once you download the Linux tar file, in terminal at V-Rep directory type: './vrep.sh'

### Python remote API
Linux already comes with Python. Create a folder for your API code (ex. vrep_code).   
Copy following files into the folder:  
*vrep/programming/remoteApiBindings/python/python/vrep.py  
vrep/programming/remoteApiBindings/python/python/vrepConst.py  
vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib*  
Create a file in the same folder where you'll store the API code. In our case, the file is  
named *test.py*. To run the code, type in terminal *python test.py*

### Inside V-Rep
In Model browser, go to robots->non-mobile. From the list, pick and drag UR3 to the scene.  
Repeat. From Model browser, go to components->grippers, pick and drag Jaco hand.  
Click on Jaco hand, while holding CTRL key, select the red origin of the gripper on UR3.  
Click Assemble (9th icon from the left on the bar). The hand should now be attached  
to the gripper. Repeat for Mico hand.
In order to add a cup to the scene, at Model browser go to household, pick and drag the cup.  

Now we are ready to code the API.

### test.py code overview
Go [here](https://gitlab.engr.illinois.edu/rmaksi2/ECE470/blob/master/FinalProject/vrep_code/test.py) to see the code. We start by importing the necessary modules.  
On line 6, all the open connections are closed, just in case. Starting on line 9, the  
connection to remote API server is established.  
Starting on line 16, we get the handles for all the joints. Since we added two UR3 robots  
it is very important to distinguish between the joints of each. To get the name of the joint  
simply go Scene hierarchy in V-Rep and click the box for one of the UR3's in order to  
show the joint names. After we get all the handles, let's start the simulation (line 65)!  
It's time to get the values for joint variables starting on line 73. Again, make sure  
to distinguish between the joints. On line 134 we start moving the joints. It should  
be clear enough to see 'how much' we move each joint in the code.  Stating on line 154,  
the hands are set to wave at the user! This piece of code is explained in the next section.  
After we perform all the moves, the code prints the current position values of the joints  
starting on line 172. We close the simulation and terminate the connection after all is done.  

### Child Scripts for Hand grippers
For this checkpoint, the hand grippers are moved by using a [Child Script](http://www.coppeliarobotics.com/helpFiles/en/childScripts.htm). There are some  
changes that needs to be performed before we can use them in 'test.py'. First, in Scene  
hierarchy, you should see a JacoHand under the first UR3. Next to it, there should be a  
piece of paper icon that needs to be double clicked. This will open the code of the child script  
for that particular JacoHand. Comment out line 19 by inserting '--' in front of it.  
Starting immediately after commented text, insert:
~~~~
sig=sim.getStringSignal('jacoHand')
if sig~=nil then

sim.clearStringSignal('jacoHand')
if sig=='true' then closing=true else closing=false end
end
~~~~
Check the reference [here](http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=1891).
Repeat for MacoHand. Do not forget to change 'jacoHand' to 'macoHand'!  
From now on, we can use out 'test.py' code to control the hands. Starting on line 154,  
hands open and close synchronously two times. Then there is a pause (for video purposes).  

<a name="Checkpoint2"></a>
## Checkpoint 2 (due 3/12/18)

### Forward Kinematics of UR3  [link to Forward Kinematics video](https://www.youtube.com/watch?v=kUiK85P_Jdc)
We implemented the basic forward kinematics of UR3 with V-REP at this checkpoint.   

First, we find the initial position and orientation of UR3 with respect to world frame.      
Then we find joint angles and implemented the matrix exponentials after which we    
calculated the final pose.  

Using the matrix logarithm of rotational matrix of the final pose, we found the angular  
velocity. With the angular velocity we can find a and theta, which we then use to find  
the quaternions.  

We referenced this Piazza post by Professor Bretl to find the equations needed:  
https://piazza.com/class/jchxn1s6tkg20r?cid=257  

We also displayed the dummy objects in the form reference frames as indication of successful  
implementation of the forward kinematics.  

The checkpoint2 file includes support files and runCode.py which is the executable file for
forward kinematics simualtion. 

<a name="Checkpoint3"></a>
## Checkpoint 3 (due 3/26/18)
Given user input we could create a goal pose, and then determine the joint angles that move the robot's end-effector 
to the desired goal pose by using inverse kinematics.

Firstly, make sure you have Remote API file and related python file in the same folder as Checkpoint3.py.

Then it will asked the user to input 6 parameters, which are intended x, y, z position of goal pose and
rotation angles in degrees around x, y , z axis, which we call a,b,c.

Secondly, we set up an dummy object which indicates the supposed end position that UR3 should move.
After enter we call the inverseKinematics Function to perform the valid joint angle calculation, the
initial guess of joint angles are randomly generalized.

We ask for user input until 3 valid positions are given, and then the simulation will stop.

The way we interfaced with V-Rep's API was the same as for Checkpoint 2, so if you're interested about how we did that
check that section.
### Inverse Kinematics of UR3
To implement the inverse kinematics for the UR3, we need the intial pose M, and an array of the 6 screw matracies.
After that first thing we do in the loop is call the simxGetJointPosition() function to get the update our theta values to
the current theta values in the simulator. 

Then we go into the looping algorithm to find theta values. 

The reference we use for this algorithm is the one discussed in Lecture 15 of the class:
~~~~
T_1in0 = getT_1in0(M, S, theta)
screwMat = logm(np.dot(T_2in0, np.linalg.inv(T_1in0)))
twist = invScrewBrac(screwMat)

counter = 0
final_pose_flag = 1

while (np.linalg.norm(twist) > 0.01):

    if(counter >= 100):
        print("Desired Pose not reachable")
        final_pose_flag = 0
        break

    J = getSpacialJacobian(S, theta)
    thetadot = np.dot(np.linalg.inv(np.dot(np.transpose(J), J) + 0.1*np.identity(6)),
    np.dot(np.transpose(J), twist)) - np.dot(np.identity(6) - np.dot(np.linalg.pinv(J), J), theta)
    theta = theta + (thetadot * k)

    T_1in0 = getT_1in0(M, S, theta)
    screwMat = logm(np.dot(T_2in0, np.linalg.inv(T_1in0)))
    twist = invScrewBrac(screwMat)

    counter = counter + 1
~~~~


<a name="Checkpoint4"></a>
## Checkpoint 4 (due 4/2/18)


### Demonstrate Collision Detection for UR3
First, make sure you have Remote API file and related Python files in the same folder as Checkpoint4.py.
You should also load the appropriate scene in the V-REP. The code does not ask for any user input. 
We predefined some of the theta values, which can be looked up in the code. Based on these values,
the collision checker is ran. You can checkout the code in Collision Detection section.

Before the robot moves, the collision checker checks if the collision will occur and will print the 
appropriate comment to the screen. Then it will move the joints. During the first "iteration",
V-REP is not running the simulation, therefore, the joints are allowed to move throught the obstacles.
During the second "iteration", V-REP is in simulation mode, therefore the joints will not be
able to move throught the obstacles no more. For both iterations, same joint configuration was used.


### Collision Detection
The implementation of collision detection:
~~~~
def collisionChecker(S, initial_points, points, r, theta):
    for i in range(len(theta)):
        p_initial = np.array([
        [initial_points[0][i]],
        [initial_points[1][i]],
        [initial_points[2][i]],
        [1]
        ])

        temp = np.identity(4)

        for j in range(i+1):
            temp = np.dot(temp, expm(screwBracForm(S[j]) * theta[j]))

        p_final = np.dot(temp, p_initial)

        points[0].append(p_final[0][0])
        points[1].append(p_final[1][0])
        points[2].append(p_final[2][0])

    p = np.block([
    [np.reshape(np.asarray(points[0]),(1,len(theta)+2))],
    [np.reshape(np.asarray(points[1]),(1,len(theta)+2))],
    [np.reshape(np.asarray(points[2]),(1,len(theta)+2))]
    ])

    for i in range(len(theta) + 2):
        point1 = np.array([
        [p[0][i]],
        [p[1][i]],
        [p[2][i]]
        ])

        for j in range(len(theta) + 2 - i):
            if(i == i+j):
                continue

            point2 = np.array([
            [p[0][j+i]],
            [p[1][j+i]],
            [p[2][j+i]]
            ])

            if(norm(point1 - point2) <= r*2):
                return 1

    return 0
~~~~



<a name="Checkpoint5"></a>
## Checkpoint 5 (due 4/9/18)
### Demonstrate Motion/Path Planing  for UR3
In this checkpoint, the user would be to asked to input desired position of the UR3. 

First, make sure you have Remote API file and related Python files in the same folder as Checkpoint5.py.
You should also load the appropriate scene in the V-REP.

In the Scene, there are two blocks which we recognize it as obstacles. In order to detect the obstacles, we implemented 
boudning sphere on the two blocks.
So the goal is to to reach to the desired pose as given the user input.

### Sampling based planner Algorithm 

1)The path planing is implemented by using sampling based planner Algorithm. We set a tree strcuture to store all the intermediate path
to connect betwwen the starting position and goal position. The tree structure has two trees: T(forward) and T(bacward), which served as
"bisection method" 

First, we check the whether the UR3 robot would encounter with self-Collision or obstacles-collision.

2)Then we sample random thetas from random uniform distribution

3)In order to genearte the one goal pose, we check whether the UR3 robot would encounter with 
self-Collision(its self joints) or obstacles-collision(bounding spheres on the two blocks).

4)if no collision, add the current position/path to T(forward), and iteratetively found new thetas
in T(forward), while set the new thetas as the child of previous theta

5)And do the same for T(backward)

6)If theta belongs to both  T(forward) and  T(backward), then we have a complete connected path from starting position to desired goal pose.


<a name="Checkpoint6"></a>
## Checkpoint 6 (due 4/16/18)

<a name="Checkpoint7"></a>
## Checkpoint 7 (due 4/23/18)

<a name="Checkpoint8"></a>
## Checkpoint 8 (due 4/30/18)

<a name="Checkpoint9"></a>
## Checkpoint 9 (due 5/7/18)
