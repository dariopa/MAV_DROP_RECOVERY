import rospy
from tkinter import *
from mav_drop_recovery.srv import SetTargetPosition, SetTargetPositionRequest
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest

# Release command for dynamixel
dynamixel_command = DynamixelCommandRequest()
dynamixel_command.command = ''
dynamixel_command.id = 1
dynamixel_command.addr_name = 'Goal_Position'

# Takeoff command
takeoff_command = SetTargetPositionRequest()
takeoff_command.command = "takeoff"

# Traverse command
traverse_command = SetTargetPositionRequest()
traverse_command.command = "traverse"

# Release command
release_command = SetTargetPositionRequest()
release_command.command = "release"

# Recovery command (NET)
recovery_net_command = SetTargetPositionRequest()
recovery_net_command.command = "recovery_net"

# Recovery command (MAGNET)
recovery_magnet_command = SetTargetPositionRequest()
recovery_magnet_command.command = "recovery_magnet"

# Homecoming command
homecoming_command = SetTargetPositionRequest()
homecoming_command.command = "homecoming"


class TrajectoryPlanner:
    def __init__(self, command, execute):
        # Anything that requires to be instantiated, goes here
        self.command = command
        self.execute = execute

    def trajectory_service_caller(self):
        self.command.execute = self.execute
        rospy.wait_for_service('/firefly/trajectory')
        try:
            plan_trajectory = rospy.ServiceProxy('/firefly/trajectory', SetTargetPosition)
            resp = plan_trajectory(self.command)
            print ("Service call to trajectory was successfull")
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

class DynamixelController:
    def __init__(self, command, steps):
        self.command = command
        self.steps = steps

    # Call service to move dynamixel
    def dynamixel_service_caller(self):
        self.command.value = self.steps
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        try:
            command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            resp = command(self.command)
            print ("Service call to Dynamixel was successfull")
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

##################################### GUI #####################################
# Open window
window = Tk()
# Define window title
window.title("Trajectory Planner")

# Define buttons
takeoff_button_visualize = Button(window, text="Visualize \nTakeoff", command=TrajectoryPlanner(takeoff_command, False).trajectory_service_caller)
traverse_button_visualize = Button(window, text="Visualize \nTraverse", command=TrajectoryPlanner(traverse_command, False).trajectory_service_caller)
release_button_visualize = Button(window, text="Visualize \nRelease", command=TrajectoryPlanner(release_command, False).trajectory_service_caller)
recovery_net_button_visualize = Button(window, text="Visualize \nRecovery Net", command=TrajectoryPlanner(recovery_net_command, False).trajectory_service_caller)
recovery_magnet_button_visualize = Button(window, text="Visualize \nRecovery Magnet", command=TrajectoryPlanner(recovery_magnet_command, False).trajectory_service_caller)
homecoming_button_visualize = Button(window, text="Visualize \nHomecoming", command=TrajectoryPlanner(homecoming_command, False).trajectory_service_caller)

takeoff_button_execute = Button(window, text="Execute \nTakeoff", command=TrajectoryPlanner(takeoff_command, True).trajectory_service_caller)
traverse_button_execute = Button(window, text="Execute \nTraverse", command=TrajectoryPlanner(traverse_command, True).trajectory_service_caller)
release_button_execute = Button(window, text="Execute \nRelease", command=TrajectoryPlanner(release_command, True).trajectory_service_caller)
recovery_net_button_execute = Button(window, text="Execute \nRecovery Net", command=TrajectoryPlanner(recovery_net_command, True).trajectory_service_caller)
recovery_magnet_button_execute = Button(window, text="Execute \nRecovery Magnet", command=TrajectoryPlanner(recovery_magnet_command, True).trajectory_service_caller)
homecoming_button_execute = Button(window, text="Execute \nHomecoming", command=TrajectoryPlanner(homecoming_command, True).trajectory_service_caller)

dynamixel_drop_button = Button(window, text="Drop GPS Box", command=DynamixelController(dynamixel_command, 24570).dynamixel_service_caller)
dynamixel_reposition_button = Button(window, text="Reposition \nDynamixel", command=DynamixelController(dynamixel_command, 10).dynamixel_service_caller)

exit_button = Button(window, text="Close", command=window.quit)

# Define labels
choose_trajectory_label = Label(window, text="Choose which trajectory you would like to visualize / execute.")
dynamixel_label = Label(window, text="Choose what to do with dynamixel.")
info_label = Label(window, text="Close the GUI.")


# Add the components in the desired order into the window
choose_trajectory_label.grid(row=0, column=1, columnspan=5, padx = 10, pady = 10)
takeoff_button_visualize.grid(row=1, column=1, padx = 10, pady = 10)
traverse_button_visualize.grid(row=1, column=2, padx = 10, pady = 10)
release_button_visualize.grid(row=1, column=3, padx = 10, pady = 10)
recovery_net_button_visualize.grid(row=1, column=4, padx = 10, pady = 10)
recovery_magnet_button_visualize.grid(row=1, column=5, padx = 10, pady = 10)
homecoming_button_visualize.grid(row=1, column=6, padx = 10, pady = 10)

takeoff_button_execute.grid(row=2, column=1, padx = 10, pady = 10)
traverse_button_execute.grid(row=2, column=2, padx = 10, pady = 10)
release_button_execute.grid(row=2, column=3, padx = 10, pady = 10)
recovery_net_button_execute.grid(row=2, column=4, padx = 10, pady = 10)
recovery_magnet_button_execute.grid(row=2, column=5, padx = 10, pady = 10)
homecoming_button_execute.grid(row=2, column=6, padx = 10, pady = 10)

dynamixel_label.grid(row=3, column=2, columnspan=4, padx=10, pady=10)
dynamixel_drop_button.grid(row=4, column=3, padx = 10, pady = 10)
dynamixel_reposition_button.grid(row=4, column=4, padx = 10, pady = 10)

info_label.grid(row=5, column=2, columnspan=4, padx = 10, pady = 10)
exit_button.grid(row=6, column=2, columnspan=4, padx = 10, pady = 10)


# Wait in the eventloop until user does something
window.mainloop()