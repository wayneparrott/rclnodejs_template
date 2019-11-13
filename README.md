# rclnode_template
A template Node.js project for creating a Robot Operating System [(ROS2)](https://index.ros.org/doc/ros2/) package implemented using JavaScript or TypeScript on top of [rclnodejs](https://github.com/RobotWebTools/rclnodejs). Includes build automation that creates and installs your node.js program such that it can function as part of a larger real-world ROS2 solution.

What this project provides:
* template ROS2 package file structure including package.xml and CMakeList.txt files
* CMake build and install_support scripts for creating ROS pkg node executable and launch files
* npm scripts for building and running your ROS pkg node executable 
* configuration instructions
* example ROS node implemented in TypeScript
* example launch file demonstrating how to launch and manage multiple pkg executables as a single unit

## Prerequisites
* [node.js]() (>=6.4.0 <11.0.0), tested with ver 10.17.0 
* [ros2] - tested with ros2 ver. dashing-diademata
* Python3 
* Your favorite coding tool - *I use CodeMix for Eclipse which has great TypeScript and Python tooling*

## Why use this project
This project is a TypeScript (JavaScript) node.js implementation of a ROS2 package. A key feature of this project is its build and installation automation that creates a ROS2 node executable script and corresponding launch file for your node.js program such that it can be run or launched from the ros2 commandline. 

## Getting Started
### 1. Things to know
#### 1a. Install as ROS2 package or use standalone
Use this project to create a package in a ROS2 workspace or as a standalone package in place of running the traditional commandline `ros2 pkg create your_package_name` 

#### 1b. Naming
Because this project is a ROS2 package the name of the directory you install it must conforms to the ROS2 package naming conventions. These are package names are lower-case characters separated by underscores, e.g., my_ros2_package*.

#### 1c. Project Configuration
As a template project you will need to edit several files, specifying the name you have choosen for the package. We'll get to this shortly.


### 2. Download or Clone the rclnode_template Git Repo
You can clone the repo, create a new github repo using this [repo as a template](https://help.github.com/en/github/creating-cloning-and-archiving-repositories/creating-a-repository-from-a-template) or download it. Regardless of the method you choose remember the name of the destination directory must conform to the ROS2 package naming conventions. 

I recommend creating a new Github repository using this project as a [template](https://help.github.com/en/github/creating-cloning-and-archiving-repositories/creating-a-repository-from-a-template). To get started click the "Use this template" button as shown below and proceed from there. Alternatively you can either clone this repo or download it to your system. 

![](https://raw.githubusercontent.com/wayneparrott/rclnodejs_template/master/github-template.png)

### 3. Edit package.xml and package.json
package.xml is a ROS2 file that specifies general package information and dependencies. Edit package.xml, near the top of the file, replace the `<name>` tag content with the name of your package.
```xml
<package format="3">
  <name>your_rospkg_name_here</name>
  ...
```

In the Node.js package.json file, replace the `name` property with the name of your ROS2 package.

```javascript
{
  "name": "your_rospkg_name_here",
  "version": "0.0.1",
  ...
  ```

### 4. Edit CMakeLists.txt
In CMakeLists.txt, near the top of the file, revise `project()` to use the name of your package
 ```
project(your_rospkg_name_here)
``` 

### 5. Edit CMakeLists_rclnodejs_project.txt
In CMakeLists_rclnodejs_project.txt file, near line #34, specify the name of the ROS2 package executable to create and install.   
```
# replace "ros_node_executable" with the name of your node
set(RUN_EXECUTABLE "rospkg_executable")
```
### 6. Install Node.js dependencies
Before installing dependent node modules, verify your are using a compatible version of node. From commandline run the following command to determine your node version.
```
   node -v
```
Your version of node must be between version 6.4 and less than version 11.0. 

Next install the project's Node.js dependencies. From commandline run:
```
   npm install
```
During the npm install process don't be alarmed by the compilation warnings that may be displayed as the rclnodejs module is installed.  

### 7. Build
Use the npm scripts for building and installing your ROS2 package. From commandline:
```
    npm run build
```
The build process does the following:
* compiles TypeScript files into the dist/ folder
* using the ROS2 colcon build system it creates the local ROS2 package internals including all custom package interfaces and messages that may be defined, (see ros/ directory)
* creates an executable script with the name you provided in CMakeLists_rclnode_project.txt (see step-5) and installs it in the local ROS2 package internals
* creates a launch directory and launch file, e.g., launch/my_rospkg_name.launch.py, and installs it in the local ROS2 package internals

### 8. Configure shell Environment
From a shell, execute the specific ros/ batch file for your environment. Here's an example for bash shell:
```
    source ros/setup.bash
```

### 9. Test
You can confirm your package installation is setup and working correction as follows.

Confirm the ROS2 package executable specified in CMakeList_rclnodejs_project.txt was created and runnable. From commandline:
```
    ros2 pkg executable <your_rospkg_name>
or
    ros2 pkg executable <your_rospkg_name> --full-path
```
You can run the executable with this command template:
```
    ros2 run <your_rospkg_name> <executable>
```
You can use the ROS2 launch system to launch the executable using the launch file that was created and istalled in step-8.
```
    ros2 launch <your_rospkg_name> <your_rospkg_name>.launch.py
```

### 10. JavaScript Configuration
If you wish to code with JavaScript only you will need to edit the start script in package.json to match your JavaScript file layout. The project is preconfigured for TypeScript with the npm build script compiling *.ts files to the 'dist/' folder and the npm start script launches the 'dist/index.js'. 


## Tips and Notes
* **When working from the commandline don't forget to setup your environment by running one of the setup.[bash, ps1, sh, zsh]**
* I've only tested this project on Linux Ubuntu 18.04 and ros2 dashing. 
* Please provide feedback on issues and improvement suggestions.
* For TypeScript support, I created the rclnodejs-types project with *.d.ts files for rclnodejs. These typings are referenced in the project tsconfig.json file shown below:

```  
"typeRoots": [
    "node_modules/@wayneparrott/rclnodejs-types",
    "node_modules/@types"
]
```




