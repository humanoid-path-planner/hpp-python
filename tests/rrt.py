from pyhpp.gepetto import Viewer
import numpy as np
from pinocchio import SE3
from pyhpp.pinocchio import Device
from pyhpp.core import Problem, Roadmap, WeighedDistance, path

# Robot configuration
urdfFilename = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
srdfFilename = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"

# Initialize robot and viewer
robot = Device.create("ur5")
viewer = Viewer("construction_set", robot)

# Add robot and obstacles to scene
viewer.addURDFToScene(0, "r0", "anchor", urdfFilename, srdfFilename, SE3.Identity())
viewer.addURDFObstacleToScene("/home/psardin/devel/nix-hpp/src/hpp-practicals/urdf/ur_benchmark/table.urdf", "table")
viewer.addURDFObstacleToScene("/home/psardin/devel/nix-hpp/src/hpp-practicals/urdf/ur_benchmark/wall.urdf", "wall")
viewer.addURDFObstacleToScene("/home/psardin/devel/nix-hpp/src/hpp-practicals/urdf/ur_benchmark/obstacles.urdf", "obstacles")

# Define initial and goal configurations
qInit = np.array([0.2, -1.57, -1.8, 0, 0.8, 0])
qGoal = np.array([1.57, -1.57, -1.8, 0, 0.8, 0])
viewer.applyConfiguration(qInit)

# Setup problem and RRT components
problem = Problem(robot)
configurationShooter = problem.configurationShooter()
steer = problem.steeringMethod()
weighedDistance = WeighedDistance.create(robot)
distance = weighedDistance.asDistancePtr_t()

# Initialize roadmap
roadmap = Roadmap.create(distance, robot)
roadmap.initNode(qInit)
roadmap.addGoalNode(qGoal)

# RRT algorithm parameters
finished = False
iter = 0
maxIter = 1000

# Main RRT loop
while not finished and iter < maxIter:
    # Extend phase
    newNodes = []
    newEdges = []
    q_rand = configurationShooter.shoot()
    iter += 1
    
    # Try to extend from each connected component
    for i in range(roadmap.numberConnectedComponents()):
        cc = roadmap.getConnectedComponent(i)
        q_near, d = roadmap.nearestNode(q_rand, cc)
        directpath = steer(q_near, q_rand)
        res, validPart, report = problem.pathValidation().validate(directpath, False)
        
        if res:
            q_new = q_rand
        else:
            q_new = validPart.end()
            
        newNodes.append(q_new)
        newEdges.append((q_near, q_new, validPart))
    
    # Add new nodes to roadmap
    for q in newNodes:
        roadmap.addNode(q)
    
    # Add new edges to roadmap
    for q1, q2, path in newEdges:
        roadmap.addEdge(q1, q2, path)
    
    # Connect phase
    for q_new in newNodes:
        for i in range(roadmap.numberConnectedComponents()):
            cc = roadmap.getConnectedComponent(i)
            q_near, d = roadmap.nearestNode(q_new, cc)
            
            if (q_near != q_new).all():
                directpath = steer(q_new, q_near)
                res, validPart, report = problem.pathValidation().validate(directpath, False)
                
                if res:
                    roadmap.addEdge(q_new, q_near, validPart)
                    break
    
    # Check if problem is solved
    nbCC = roadmap.numberConnectedComponents()
    if nbCC == 1:
        print('Problem solved!')
        finished = True

# Compute and display final path
if finished:
    path = problem.target().computePath(roadmap)
    viewer.displayPath(path)
else:
    print(f"Maximum iterations ({maxIter}) reached without finding solution")