import numpy as np
from pyhpp.constraints import ComparisonType
from pyhpp.core.path_optimization import SplineGradientBasedAbstractB3
from pyhpp.core.path import SplineB3 as Spline
from pyhpp.core.path_optimization import LinearConstraint


class CostFunction:
    def __init__(self, robot, order):
        self.order = order
        self.robot = robot

    def value(self, splines):
        return sum(s.squaredNormIntegral(self.order) for s in splines)

    def jacobian(self, splines):
        r = splines[0].NbCoeffs * self.robot.numberDof()
        J = np.empty((splines.size() * r, 1))
        n = 0
        for s in splines:
            s.squaredNormIntegralDerivative(self.order, J[n : n + r, :])
            n += r
        return J.T

    def hessian(self, splines):
        r = splines[0].NbCoeffs * self.robot.numberDof()
        Ic = np.empty((splines[0].NbCoeffs, splines[0].NbCoeffs))
        H = np.zeros((r * splines.size(), r * splines.size()))

        for k, s in enumerate(splines):
            s.squaredNormBasisFunctionIntegral(self.order, Ic)
            Ic *= 2
            paramSize = s.parameterSize()
            shift = k * s.NbCoeffs * paramSize
            for i in range(s.NbCoeffs):
                ic = shift + i * paramSize
                for j in range(s.NbCoeffs):
                    jc = shift + j * paramSize
                    H[ic : ic + paramSize, jc : jc + paramSize] = Ic[i, j] * np.eye(
                        paramSize
                    )
        return H


class NonLinearSplineGradientBasedB3(SplineGradientBasedAbstractB3):
    def __init__(self, problem):
        super(NonLinearSplineGradientBasedB3, self).__init__(problem)
        self.problem = problem

    def optimize(self, path):
        print("NonLinearSplineGradientBasedB3::optimize")
        checkJointBound = True

        robot = self.problem.robot()
        rDof = robot.numberDof()

        # 1
        splines = SplineGradientBasedAbstractB3.Splines()
        self.appendEquivalentSpline(path, splines)
        if splines.size() == 1:
            return self.buildPathVector(splines)

        self.initializePathValidation(splines)

        # 2
        nParameters = len(splines) * Spline.NbCoeffs
        # Order 1 -> max continuity: 0
        # Order 3 -> max continuity: 1
        SplineOrder = 3
        MaxContinuityOrder = int((SplineOrder - 1) / 2)
        # For the sake of the example, use C^0 continuity, not C^1
        orderContinuity = MaxContinuityOrder
        # orderContinuity = 0

        constraint = LinearConstraint(nParameters * rDof, 0)
        solvers = self.SplineOptimizationDatas(
            splines.size(), self.SplineOptimizationData(rDof)
        )

        # TODO
        # addProblemConstraints (input, splines, constraint, solvers);
        self.addContinuityConstraints(splines, orderContinuity, solvers, constraint)

        validations, parameterizations = self.createConstraints(splines)
        value, jacobian = self.initValueAndJacobian(
            splines, validations, parameterizations
        )
        linearizedConstraint = LinearConstraint(jacobian.shape[1], jacobian.shape[0])
        lcReduced = LinearConstraint(0, 0)
        # self.computeValue    (value, splines, validations, parameterizations)
        # self.computeJacobian (jacobian, splines, validations, parameterizations)
        # self.computeLinearizedConstraint (linearizedConstraint, value, jacobian, splines, validations, parameterizations)

        # 3
        collision = LinearConstraint(nParameters * rDof, 0)
        # collisionFunctions = CollisionFunctions ();

        # 4
        # TODO: Initialize cost function.
        cost = CostFunction(robot, 1)
        print("Current {cost.value(splines)}")

        # 5
        feasible = constraint.decompose(True, True)

        self.checkConstraint(splines, constraint, msg="continuity")

        collisionReduced = LinearConstraint(constraint.PK.shape[0], 0)
        constraint.reduceConstraint(collision, collisionReduced)

        boundConstraint = LinearConstraint(nParameters * rDof, 0)
        if checkJointBound:
            self.jointBoundConstraint(splines, boundConstraint)
            self.checkConstraint(splines, boundConstraint, msg="bounds", ineq=True)
            # if (!this->validateBounds(splines, boundConstraint).empty())
            # throw std::invalid_argument("Input path does not satisfy joint bounds");
        boundConstraintReduced = LinearConstraint(boundConstraint.PK.shape[0], 0)
        constraint.reduceConstraint(boundConstraint, boundConstraintReduced, False)

        costLinearized = LinearConstraint(nParameters * rDof, 1)
        clReduced = LinearConstraint(0, 1)
        constraint.reduceConstraint(costLinearized, clReduced)

        finalLinearProblem = LinearConstraint(0, 1)
        ok = False
        iter = 0
        while not ok:
            v = cost.value(splines)
            print("Iter {iter}, cost: {v}")

            # Compute linearized cost
            parameters = np.empty((nParameters * rDof, 1))
            self.updateParameters(parameters, splines)
            costLinearized.J = cost.jacobian(splines)
            costLinearized.v = costLinearized.J.dot(parameters) - np.matrix([[v]])
            constraint.reduceConstraint(costLinearized, clReduced)

            # Compute decomposition.
            self.computeLinearizedConstraint(
                linearizedConstraint,
                value,
                jacobian,
                splines,
                validations,
                parameterizations,
            )
            print("Linearized constraints")
            constraint.reduceConstraint(linearizedConstraint, lcReduced, True)
            print("reduction")
            lcReduced.decompose(True, True)
            print("decompose")

            lcReduced.reduceConstraint(clReduced, finalLinearProblem, False)
            print("reduction")
            finalLinearProblem.decompose(True, True)  # TODO need only xStar, not PK

            # Compute descent direction.
            lcReduced.computeSolution(finalLinearProblem.xStar)
            print("computeSolution")
            constraint.computeSolution(lcReduced.xSol)

            self.integrate(splines, constraint.xSol)

            iter += 1
            if iter == 5:
                ok = True

        return self.buildPathVector(splines)

    def integrate(self, splines, dir):
        alpha = 0.5
        row = 0
        for s in splines:
            n = s.parameterSize() * s.NbCoeffs
            s.parameterIntegrate(alpha * dir[row : row + n])
            row += n

    def createConstraints(self, splines):
        # Functions of the form f(v_k) == 0
        validations = [[] for s in splines]
        # Functions of the form g(v_k) == g(v_{k+1})
        parameterizations = [[] for s in splines]
        for k, s in enumerate(splines):
            # Extend function to
            c = s.constraints()
            if c:
                cp = c.configProjector()
                if cp:
                    ncs = cp.numericalConstraints()
                    ljs = cp.lockedJoints()
                    for nc in ncs:
                        if ComparisonType.Equality in nc.comparisonType():
                            parameterizations[k].append(nc)
                        else:
                            validations[k].append(nc)
                    for nc in ljs:
                        if ComparisonType.Equality in nc.comparisonType():
                            parameterizations[k].append(nc)
                        else:
                            validations[k].append(nc)
                else:
                    print(f"{k} no config projector")
            else:
                print(f"{k} no constraints")
        return validations, parameterizations

    def initValueAndJacobian(self, splines, validations, parameterizations):
        vRows = 0
        JRows = JCols = 0
        for k, (s, v, p) in enumerate(zip(splines, validations, parameterizations)):
            for f in v:
                vRows += f.function().outputSize() * 2
                JRows += f.function().outputDerivativeSize() * 2
            for f in p:
                vRows += f.function().outputDerivativeSize()
                JRows += f.function().outputDerivativeSize()

            JCols += s.parameterSize() * s.NbCoeffs

        return np.zeros((vRows, 1)), np.zeros((JRows, JCols))

    def computeValue(self, value, splines, validations, parameterizations):
        i = 0
        for k, (s, v, p) in enumerate(zip(splines, validations, parameterizations)):
            qinit = s.initial()
            qend = s.end()

            for f in v:
                e = i + f.function().outputSize()
                value[i:e] = f.function()(qinit).vector()
                i = e

                e = i + f.function().outputSize()
                value[i:e] = f.function()(qend).vector()
                i = e

            for f in p:
                e = i + f.function().outputDerivativeSize()
                value[i:e] = f.function()(qinit) - f.function()(qend)
                i = e

    def computeJacobian(self, J, splines, validations, parameterizations):
        ir = ic = 0
        for k, (s, v, p) in enumerate(zip(splines, validations, parameterizations)):
            qinit = s.initial()
            qend = s.end()

            ps = s.parameterSize()
            ec = ic + s.NbCoeffs * ps
            ics = [ic + i * ps for i in range(s.NbCoeffs + 1)]

            # Only the endpoints and not the other points ?
            for f in v:
                er = ir + f.function().outputDerivativeSize()
                Jfunction = f.function().J(qinit)
                paramDerInit = s.parameterDerivativeCoefficients(s.paramRange().first)
                for l in range(s.NbCoeffs):
                    J[ir:er, ics[l] : ics[l + 1]] = Jfunction * paramDerInit[l, 0]
                ir = er

                er = ir + f.function().outputDerivativeSize()
                Jfunction = f.function().J(qend)
                paramDerEnd = s.parameterDerivativeCoefficients(s.paramRange().second)
                for l in range(s.NbCoeffs):
                    J[ir:er, ics[l] : ics[l + 1]] = Jfunction * paramDerEnd[l, 0]
                ir = er

            for f in p:
                er = ir + f.function().outputDerivativeSize()
                JfunctionInit = f.function().J(qinit)
                JfunctionEnd = f.function().J(qend)
                paramDerInit = s.parameterDerivativeCoefficients(s.paramRange().first)
                paramDerEnd = s.parameterDerivativeCoefficients(s.paramRange().second)
                # J[ir:er, ics[0]:ics[1]] = f.function().J(s.end()) * s.parameterDerivativeCoefficients(s.paramRange().second)
                for l in range(s.NbCoeffs):
                    J[ir:er, ics[l] : ics[l + 1]] = (
                        JfunctionInit * paramDerInit[l, 0]
                        - JfunctionEnd * paramDerEnd[l, 0]
                    )
                ir = er

        return J

    def computeLinearizedConstraint(
        self, constraint, value, J, splines, validations, parameterizations
    ):
        self.computeValue(value, splines, validations, parameterizations)
        self.computeJacobian(J, splines, validations, parameterizations)

        s = splines[0]
        parameters = np.empty((splines.size() * s.parameterSize() * s.NbCoeffs, 1))
        self.updateParameters(parameters, splines)

        constraint.b = J.dot(parameters) - value
        constraint.J = J
        constraint.decompose(False, False)

    def checkConstraint(self, splines, constraints, ineq=False, msg=""):
        x = np.empty((0))
        for s in splines:
            x = np.concatenate((x, s.rowParameters()))
        res = constraints.J.dot(x) - constraints.b
        if ineq:
            ok = all(res >= 0)
        else:
            ok = np.linalg.norm(res) < 1e-3
        if not ok:
            print(f"Constraint {msg} not satisfied: {res.T}")
        return ok, res

    @staticmethod
    def create(problem):
        print("NonLinearSplineGradientBasedB3.create called!")
        return NonLinearSplineGradientBasedB3(problem)
