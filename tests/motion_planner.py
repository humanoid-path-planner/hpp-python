class MotionPlanner:
    def __init__(self, robot, ps):
        self.robot = robot
        self.ps = ps

    def solveBiRRT(self, maxIter=float("inf")):
        self.ps.prepareSolveStepByStep()
        finished = False

        # In the framework of the course,
        # we restrict ourselves to 2 connected components.
        nbCC = self.ps.numberConnectedComponents()
        if nbCC != 2:
            raise Exception("There should be 2 connected components.")

        iter = 0
        ps = self.ps
        robot = self.robot
        while True:
            # RRT begin
            # Extend
            newNodes = list()
            newEdges = list()
            q_rand = robot.shootRandomConfig()
            for i in range(ps.numberConnectedComponents()):
                q_near, d = ps.getNearestConfig(q_rand,i)
                res, pid, msg = ps.directPath(q_near, q_rand, True)
                if res:
                    q_new = q_rand
                else:
                    q_new = ps.configAtParam(pid, ps.pathLength(pid))
                newNodes.append(q_new)
                newEdges.append((q_near, q_new, pid))
            for q in newNodes:
                ps.addConfigToRoadmap(q)
            for q1, q2, pid in newEdges:
                ps.addEdgeToRoadmap(q1, q2, pid, True)
            # connect
            for q_new in newNodes:
                for i in range(ps.numberConnectedComponents()):
                    q_near, d = ps.getNearestConfig(q_new, i)
                    # if q_near == q_new, q_new is in this connected component
                    if q_near != q_new:
                        res, pid, msg = ps.directPath(q_new, q_near, True)
                        if res:
                            ps.addEdgeToRoadmap(q_new, q_near, pid, True)
                            print('finished')
                            break
            # RRT end
            # Check if the problem is solved.
            nbCC = self.ps.numberConnectedComponents()
            if nbCC == 1:
                # Problem solved
                finished = True
                break
            iter = iter + 1
            if iter > maxIter:
                break
        if finished:
            self.ps.finishSolveStepByStep()
            return self.ps.numberPaths() - 1

    def solvePRM(self):
        self.ps.prepareSolveStepByStep()
        # PRM begin
        # PRM end
        self.ps.finishSolveStepByStep()
