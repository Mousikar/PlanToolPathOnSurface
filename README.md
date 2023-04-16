# PlanToolPathOnSurface
- Executes a scan path with a depth camera
- Reconstructs the surface of the objects observed by the camera
- Plan tool path on the reconstructed surface
- Plans robot motions the tool path
- Executes the process path

## run
`roscore`

`rosrun pathplan parallel_finish_zig.py`

`rostopic echo /chatter`
