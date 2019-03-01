# rescue-major-software
Rescue Major Software




### Running ros:
when in catkin_ws:
- run "catkin_make"
- run ". ~/rescue-major-software/catkin_ws/devel/setup.bash"
- run "roscore"
- run "rosrun nuc_main nuc_main.py"



### Running the GUI:
- First run:
    - After all the above, ```rosrun gui gui```
    - This will register our gui with rqt
- Subsequent runs:
    - either:
        - ```rqt --standalone gui```
        - ```rqt``` and select RMRC GUI under the plugins window



### Connecting to the NUC
- Startup the NUC
- Wait about 3-4 min for the system to start up
- SSH into the machine
	- ```bash
	  ssh $(curl http://ckl41.user.srcf.net/ipaddress_broadcast.php)
	  ```
