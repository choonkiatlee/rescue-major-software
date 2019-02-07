# rescue-major-software
Rescue Major Software

Running ros:
when it catkin_ws:
- run "catkin_make"
- run ". ~/rescue-major-software/catkin_ws/devel/setup.bash"
- run "roscore"
- run "rosrun nuc_main nuc_main.py"



Running the GUI:
- First run:
    - After all the above, ```bash rosrun gui gui```
    - This will register our gui with rqt
- Subsequent runs:
    - either:
        - ```bash rqt --standalone gui```
        - ```bash rqt``` and select RMRC GUI under the plugins window