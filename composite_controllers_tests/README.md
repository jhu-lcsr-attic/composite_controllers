Composite Controllers Tests
===========================

Effort Sum Test / Example
-------------------------

This example involves loading a controller which adds the effort commands of
multiple subcontrollers. Each of the subcontrollers commands 1.0 units of
effort, so the commanded effort for the robot should be 2.0 on each joint.

```shell
# Start the test program (the commanded effort will be printed to console)
roslaunch composite_controllers_tests effort_sum.launch

# Load the effort sum & start it
rosservice call /controller_manager/load_controller effort_sum_controller
rosservice call /controller_manager/switch_controller "{start_controllers: ['effort_sum_controller'], stop_controllers: [], strictness: 0}" 

# Add the subcontrollers
rosservice call /effort_sum_controller/add_subcontroller c1
rosservice call /effort_sum_controller/add_subcontroller c2

# Load test controllers into the subcontrollers
rosservice call /effort_sum_controller/c1/controller_manager/load_controller effort_test_controller
rosservice call /effort_sum_controller/c2/controller_manager/load_controller effort_test_controller

# Start the test controllers
rosservice call /effort_sum_controller/c1/controller_manager/switch_controller "{start_controllers: ['effort_test_controller'], stop_controllers: [], strictness: 0}" 
rosservice call /effort_sum_controller/c2/controller_manager/switch_controller "{start_controllers: ['effort_test_controller'], stop_controllers: [], strictness: 0}" 
```
