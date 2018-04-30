from pyhpp.pinocchio import Device, urdf

robot = Device.create('ur3')
urdf.loadRobotModel (robot, "anchor", "ur_description", "ur3", "_gripper", "_gripper")
