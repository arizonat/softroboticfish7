In order to run the fish controller, execute the following commands:
rosrun fishstatecontroller ObjectTracker.py (starts object tracking with raspicam_node)
rosrun fishstatecontroller FishStateMachine.py (starts the state machine that controls the fish at a high level)
rosrun fishtatecontroller auto_controller.launch (launches the PID nodes associated with heading, pitch, and thrust)
rosrun fishstatecontroller serial_node.py (send the fish actuator commands to the mbed) 
