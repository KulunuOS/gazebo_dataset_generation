This is the official source code for Automatic Dataset Generation From CAD for Vision-Based Grasping, ICAR, 2021.
PDF: https://research.tuni.fi/uploads/2021/10/5f29a62c-icar_2021_dataset.pdf
video: https://youtu.be/DwlWrxc3Wis


## Introduction
This is a gazebo based pipeline to generate simulated rgb/depth image datasets. A Simulated dataset is a convienient approach to obtain customized data for deep learning projects, compared to real data. In this approach, A Kinect camera is simulated around the origin of the gazebo world (view sampling) according to a pre-defined algorithm and images are captured and saved.

![View Sampling procedure](/assets/images/view_sampling.jpg "View Sampling procedure")

Watch the video on [Youtube](https://www.youtube.com/watch?v=Fa-J-9h2a0w​ "Dataset Generation in Gazebo Simulator").

## Requirements

- Ubuntu 20.04 and ROS Noetic
- Gazebo multi-robot simulator, version 11.5.1
- Anaconda distribution 
- Python 3.6 

## Installation
1. Create a catkin_ws in your working directory :
    ~~~
    $ mkdir -p ~/catrkin_ws/src
    ~~~
2. Clone the repo into your src folder :
    ~~~
    $ cd catkin_ws/src
    $ git clone --branch object_classification https://github.com/KulunuOS/gazebo_dataset_generation.git
    ~~~

3. Build the workspace
    ~~~
    $ cd ..
    $ catkin_make
    ~~~
4. roslaunch the gazebo world
    ~~~
    $ source devel/setup.bash 
    $ roslaunch data_generation metrics.launch
    ~~~ 
5. Install dependencies and activate conda environment. ( you need to have anaconda installed and configured )
    ~~~
    $ cd src
    $ conda env create -f env.yml
    $ conda activate open3d
    ~~~

6. Ḿodify the gazebo world according to the preference of your dataset.
    
    Include the mesh files, model.sdf, model.config files of the objects in your dataset in data_generation/models folder
    
    Include the following in data_generation/Worlds/metrics.world file according to preference

    An example of loading the object 'left_gear' on to the table is as below :
    ~~~
    <include>
      <uri>model://left_gear</uri>
      <name>left_gear</name>
      <pose>0 0 0.84 0 0 0</pose>
    </include> 
    ~~~


## Data Generation

1. launch the gazebo world again as in step 4 and check if the objects are placed according to your preference.

2. Run the generating script in a seperate terminal in parallel with gazebo world launched previously  
    ~~~
    $ cd ~/data_generation/gen_script                          
    $ python render.py models <link_name>
    ~~~
3. The rgb and depth files will be saved to /data_generation/gen_script directory in the folders /rgb and /depth respectively
