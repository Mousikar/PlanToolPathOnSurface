# PlanToolPathOnSurface
- Executes a scan path with a depth camera
- Reconstructs the surface of the objects observed by the camera
- Plan tool path on the reconstructed surface
- Plans robot motions the tool path
- Executes the process path

## run base on **opencamlib**
`roscore`

`rosrun pathplan parallel_finish_zig.py`

![image](https://user-images.githubusercontent.com/103837402/236463557-9921f625-7f61-4e71-b0cd-cd5be4b25b89.png)


`rostopic echo /chatter`

`rosrun pathplan pathplanning.py`

![image](https://user-images.githubusercontent.com/103837402/236463069-b30920ad-6021-4a22-a16f-33e1eff5e6b2.png)


## run base on open3d
`roscore`

`rosrun pathplan pathplan.py`

`rostopic echo /pathplanning`
![image](https://user-images.githubusercontent.com/103837402/236463330-c2aebbb6-f872-49f8-9edd-7111fc1a900a.png)

![image](https://user-images.githubusercontent.com/103837402/236463294-47126606-98d0-4e26-acad-399e7bc79de9.png)
