# pyhpp.core — documentation status

## Legend

- ✅ **Documented** — docstring present in the binding
- ❌ **No C++ doc** — C++ method has no `///` and no generated doc
- ⚠️ **Python-only** — no direct C++ equivalent → wrapper or modified signature
- ⚠️ **Signature diff.** — Python wrapper hides output params → `DocClassMethod` would trigger a Boost.Python static_assert

---

## path.cc

### Path

| Python method | Status | Note |
|---|---|---|
| `__call__` (×2) | ✅ | Inline strings — `(q, success)` and in-place form |
| `eval` (×2) | ✅ | Same inline strings as `__call__` |
| `derivative` | ✅ | Inline string — returns numpy vector |
| `constraints` | ✅ | Inline string — no `///` in `path.hh` |
| `copy` | ✅ | `DocClassMethod(copy)` |
| `extract` | ✅ | `DocClassMethod(extract)` |
| `timeRange` | ✅ | `DocClassMethod(timeRange)` |
| `reverse` | ✅ | `DocClassMethod(reverse)` |
| `paramRange` | ✅ | `DocClassMethod(paramRange)` |
| `length` | ✅ | `DocClassMethod(length)` |
| `initial` | ✅ | `DocClassMethod(initial)` |
| `end` | ✅ | `DocClassMethod(end)` |
| `outputSize` | ✅ | `DocClassMethod(outputSize)` |
| `outputDerivativeSize` | ✅ | `DocClassMethod(outputDerivativeSize)` |
| `StraightPath.__init__` | ❌ No C++ doc | Factory wrapper — no `///` on `StraightPath::create` |

---

## problem.cc

### Problem

| Method / property | Status | Note |
|---|---|---|
| `robot` | ✅ | `DocClassMethod(robot)` |
| `setParameter` | ✅ | `DocClassMethod(setParameter)` |
| `getParameter` | ✅ | `DocClassMethod(getParameter)` |
| `addConfigValidation` | ✅ | `DocClassMethod(addConfigValidation)` |
| `clearConfigValidations` | ✅ | `DocClassMethod(clearConfigValidations)` |
| `initConfig` | ✅ | `DocClassMethod(initConfig)` |
| `addGoalConfig` | ✅ | `DocClassMethod(addGoalConfig)` |
| `resetGoalConfigs` | ✅ | `DocClassMethod(resetGoalConfigs)` |
| `steeringMethod` (getter/setter) | ✅ | Inline strings — no `///` in `problem.hh` |
| `configValidation` (getter/setter) | ✅ | Inline strings |
| `pathValidation` (getter/setter) | ✅ | Inline strings |
| `pathProjector` (getter/setter) | ✅ | Inline strings |
| `distance` (getter/setter) | ✅ | Inline strings |
| `target` (getter/setter) | ✅ | Inline strings |
| `configurationShooter` (getter/setter) | ✅ | Inline strings |
| `errorThreshold` (def_readwrite) | ✅ | Inline string — Python-only field |
| `maxIterProjection` (def_readwrite) | ✅ | Inline string — Python-only field |
| `addPartialCom` | ✅ | Inline string |
| `getPartialCom` | ✅ | Inline string |
| `createRelativeComConstraint` | ✅ | Inline string |
| `createTransformationConstraint` (×2) | ✅ | Inline string |
| `setConstantRightHandSide` | ✅ | Inline string |
| `applyConstraints` | ✅ | Inline string — returns `(success, projected_config, residual_error)` |
| `isConfigValid` | ✅ | Inline string — returns `(valid, report)` |
| `setConstraints` | ✅ | Inline string |
| `getConstraints` | ✅ | Inline string |
| `setRightHandSideFromConfig` | ✅ | Inline string |
| `addNumericalConstraintsToConfigProjector` (×2) | ✅ | Inline strings |
| `createComBetweenFeet` | ✅ | Inline string |
| `directPath` | ✅ | Inline string — returns `(valid, path, report)` |

---

## roadmap.cc

### Roadmap

| Python method | Status | Note |
|---|---|---|
| `clear` | ✅ | `DocClassMethod(clear)` |
| `nodesWithinBall` | ✅ | `DocClassMethod(nodesWithinBall)` |
| `addEdges` | ✅ | `DocClassMethod(addEdges)` |
| `merge` | ✅ | `DocClassMethod(merge)` |
| `insertPathVector` | ✅ | `DocClassMethod(insertPathVector)` |
| `addGoalNode` | ✅ | `DocClassMethod(addGoalNode)` |
| `resetGoalNodes` | ✅ | `DocClassMethod(resetGoalNodes)` |
| `pathExists` | ✅ | `DocClassMethod(pathExists)` |
| `goalNodes` | ✅ | `DocClassMethod(goalNodes)` |
| `distance` | ✅ | `DocClassMethod(distance)` |
| `addNode` | ✅ | Inline string |
| `nearestNode` (×4) | ✅ | Inline strings — returns `(config, minDistance)` |
| `nearestNodes` (×2) | ✅ | Inline strings |
| `addNodeAndEdges` | ✅ | Inline string |
| `addNodeAndEdge` | ✅ | Inline string |
| `addEdge` (×2) | ✅ | Inline strings |
| `nodes` | ✅ | Inline string |
| `nodesConnectedComponent` | ✅ | Inline string |
| `initNode` (×2) | ✅ | Inline strings |
| `connectedComponents` | ✅ | Inline string |
| `numberConnectedComponents` | ✅ | Inline string |
| `getConnectedComponent` | ✅ | Inline string |
| `connectedComponentOfNode` | ✅ | Inline string |

---

## path-planner.cc

### PathPlanner

| Python method | Status | Note |
|---|---|---|
| `roadmap` | ✅ | `DocClassMethod(roadmap)` |
| `problem` | ✅ | `DocClassMethod(problem)` |
| `startSolve` | ✅ | `DocClassMethod(startSolve)` |
| `solve` | ✅ | `DocClassMethod(solve)` |
| `tryConnectInitAndGoals` | ✅ | `DocClassMethod(tryConnectInitAndGoals)` |
| `oneStep` | ✅ | `DocClassMethod(oneStep)` |
| `finishSolve` | ✅ | `DocClassMethod(finishSolve)` |
| `interrupt` | ✅ | `DocClassMethod(interrupt)` |
| `stopWhenProblemIsSolved` | ✅ | `DocClassMethod(stopWhenProblemIsSolved)` |
| `computePath` | ✅ | `DocClassMethod(computePath)` |
| `maxIterations` (getter) | ✅ | `DOC_PP_MAXITER_GET` from `path-planner.hh` |
| `maxIterations` (setter) | ✅ | `DOC_PP_MAXITER_SET` |
| `timeOut` (getter) | ✅ | `DOC_PP_TIMEOUT_GET` from `path-planner.hh` |
| `timeOut` (setter) | ✅ | `DOC_PP_TIMEOUT_SET` |

---

## path-optimizer.cc

### PathOptimizer

| Python method | Status | Note |
|---|---|---|
| `problem` | ✅ | `DocClassMethod(problem)` |
| `optimize` | ✅ | `DocClassMethod(optimize)` |
| `interrupt` | ✅ | `DocClassMethod(interrupt)` |
| `maxIterations` | ✅ | `DocClassMethod(maxIterations)` (setter-only, safe) |
| `timeOut` | ✅ | `DocClassMethod(timeOut)` (setter-only, safe) |

### TrapezoidalTimeParameterization

| Python property | Status | Note |
|---|---|---|
| `maxVelocity` | ✅ | `DOC_TTP_MAXVEL` from `trapezoidal-time-parameterization.hh` |
| `maxAcceleration` | ✅ | `DOC_TTP_MAXACC` |
| `minimumDuration` | ✅ | `DOC_TTP_MINDUR` |

### SimpleTimeParameterization

| Python property | Status | Note |
|---|---|---|
| `safety` | ✅ | `DOC_STP_SAFETY` — from `///` in `simple-time-parameterization.hh` |
| `order` | ✅ | `DOC_STP_ORDER` |
| `maxAcceleration` | ✅ | `DOC_STP_MAXACC` |

### SplineGradientBased (×3 templates)

| Python property | Status | Note |
|---|---|---|
| `alphaInit` | ✅ | `DOC_SGB_ALPHAINIT` — from `///` in `spline-gradient-based.hh` |
| `alwaysStopAtFirst` | ✅ | `DOC_SGB_ALWAYSSTOPFIRST` |
| `costOrder` | ✅ | `DOC_SGB_COSTORDER` |
| `usePathLengthAsWeights` | ✅ | `DOC_SGB_USEPATHLENGTH` |
| `reorderIntervals` | ✅ | `DOC_SGB_REORDERINTERVALS` |
| `linearizeAtEachStep` | ✅ | `DOC_SGB_LINEARIZE` |
| `checkJointBound` | ✅ | `DOC_SGB_CHECKJOINTBOUND` |
| `returnOptimum` | ✅ | `DOC_SGB_RETURNOPTIMUM` |
| `costThreshold` | ✅ | `DOC_SGB_COSTTHRESHOLD` |
| `guessThreshold` | ✅ | `DOC_SGB_GUESSTHRESHOLD` |
| `QPAccuracy` | ✅ | `DOC_SGB_QPACCURACY` |

---

## steering-method.cc

### SteeringMethod

| Python method | Status | Note |
|---|---|---|
| `__call__` | ✅ | Inline string |
| `steer` | ✅ | `DocClassMethod(steer)` |
| `problem` | ✅ | `DocClassMethod(problem)` |
| `constraints` (setter) | ✅ | `DocClassMethod(constraints)` |
| `constraints` (getter) | ✅ | Inline string (getter/setter pair → `DocClassMethod` unsafe on getter) |

---

## node.cc

### Node

| Python method | Status | Note |
|---|---|---|
| `addOutEdge` | ✅ | `DocClassMethod(addOutEdge)` |
| `addInEdge` | ✅ | `DocClassMethod(addInEdge)` |
| `connectedComponent` (getter) | ✅ | Inline string — no `///` in `node.hh` |
| `connectedComponent` (setter) | ✅ | Inline string |
| `outEdges` | ✅ | `DocClassMethod(outEdges)` |
| `inEdges` | ✅ | `DocClassMethod(inEdges)` |
| `isOutNeighbor` | ✅ | `DocClassMethod(isOutNeighbor)` |
| `isInNeighbor` | ✅ | `DocClassMethod(isInNeighbor)` |
| `configuration` | ✅ | `DocClassMethod(configuration)` |

---

## constraint.cc

### Constraint

| Python method | Status | Note |
|---|---|---|
| `name` | ✅ | `DocClassMethod(name)` |
| `apply` | ✅ | Inline string — modifies `q` in-place |
| `isSatisfied` (×2) | ✅ | Inline strings |
| `copy` | ✅ | Inline string |

### ConstraintSet

| Python method | Status | Note |
|---|---|---|
| `addConstraint` | ✅ | `DocClassMethod(addConstraint)` |
| `configProjector` | ✅ | `DocClassMethod(configProjector)` |

### ConfigProjector

| Method / property | Status | Note |
|---|---|---|
| `solver` | ✅ | `DocClassMethod(solver)` |
| `lineSearchType` (add_property) | ✅ | `DOC_CP_LINESEARCHTYPE` |
| `add` | ✅ | `DocClassMethod(add)` |
| `lastIsOptional` (getter/setter) | ✅ | Inline strings — no `///` in `config-projector.hh` |
| `maxIterations` (getter) | ✅ | `DOC_CP_MAXITER_GET` |
| `maxIterations` (setter) | ✅ | `DOC_CP_MAXITER_SET` |
| `errorThreshold` (getter) | ✅ | `DOC_CP_ERRTHRESH_GET` |
| `errorThreshold` (setter) | ✅ | `DOC_CP_ERRTHRESH_SET` |
| `residualError` | ✅ | `DocClassMethod(residualError)` |
| `sigma` | ✅ | `DocClassMethod(sigma)` |
| `setRightHandSideFromConfig` | ✅ | Inline string |
| `setRightHandSideOfConstraint` | ✅ | Inline string |
| `numericalConstraints` | ✅ | Inline string |

---

## distance.cc

### Distance

| Python method | Status | Note |
|---|---|---|
| `compute` | ✅ | `DocClassMethod(compute)` |

### WeighedDistance

| Python method | Status | Note |
|---|---|---|
| `asDistancePtr_t` | ✅ | Inline string |
| `getWeights` | ✅ | Inline string |
| `setWeights` | ✅ | Inline string |

---

## connected-component.cc

### ConnectedComponent

| Python method | Status | Note |
|---|---|---|
| `nodes` | ✅ | `DocClassMethod(nodes)` |
| `reachableFrom` | ✅ | `DocClassMethod(reachableFrom)` |
| `reachableTo` | ✅ | `DocClassMethod(reachableTo)` |
| `__eq__` | ✅ | Inline string |

---

## configuration-shooter.cc

### ConfigurationShooter

| Python method | Status | Note |
|---|---|---|
| `shoot` | ✅ | `DocClassMethod(shoot)` |

---

## config-validation.cc

### ConfigValidation

| Python method | Status | Note |
|---|---|---|
| `validate` | ✅ | `DocClassMethod(validate)` via `PYHPP_DEFINE_METHOD2` |
| `validate` (py tuple) | ✅ | Inline string — returns `(bool, report)` |

### ConfigValidations

| Python method | Status | Note |
|---|---|---|
| `add` | ✅ | `DocClassMethod(add)` via `PYHPP_DEFINE_METHOD2` |
| `numberConfigValidations` | ✅ | `DocClassMethod(numberConfigValidations)` |
| `clear` | ✅ | Inline string — no `///` in `config-validations.hh` |

---

## parameter.cc

### Parameter

| Python method | Status | Note |
|---|---|---|
| `boolValue` | ✅ | Inline string |
| `intValue` | ✅ | Inline string |
| `floatValue` | ✅ | Inline string |
| `stringValue` | ✅ | Inline string |
| `vectorValue` | ✅ | Inline string |
| `matrixValue` | ✅ | Inline string |
| `value` | ✅ | Inline string — converts to Python object by type |
| `create_bool` | ✅ | Inline string — factory for `bool` |

---

## path-projector.cc

### PathProjector

| Python method | Status | Note |
|---|---|---|
| `apply` | ✅ | `DocClassMethod(apply)` |
| `apply` (py tuple) | ✅ | Inline string — returns `(bool, projPath)` |
| `NoneProjector` | ✅ | Inline string |
| `ProgressiveProjector` | ✅ | Inline string |
| `DichotomyProjector` | ✅ | Inline string |
| `GlobalProjector` | ✅ | Inline string |
| `RecursiveHermiteProjector` | ✅ | Inline string |

---

## path-validation.cc

### PathValidation

| Python method | Status | Note |
|---|---|---|
| `validate` | ✅ | `DocClassMethod(validate)` |
| `validate` (py tuple) | ✅ | Inline string — returns `(bool, validPart, report)` |
| `validateConfiguration` | ✅ | Inline string — returns `(bool, report)` |
| `Discretized` / `DiscretizedCollision` | ✅ | Inline string |
| `DiscretizedJointBound` | ✅ | Inline string |
| `DiscretizedCollisionAndJointBound` | ✅ | Inline string |
| `Progressive` | ✅ | Inline string |
| `Dichotomy` | ✅ | Inline string |

---

## problem-target.cc

### ProblemTarget

No methods exposed.

---

## reports.cc

### ValidationReport / CollisionValidationReport / JointBoundValidationReport / PathValidationReport

| Exposed | Status | Note |
|---|---|---|
| All `def_readonly`/`def_readwrite` fields | ❌ No C++ doc | No `///` in the corresponding headers |

---

## path/vector.cc

### PathVector (exposed as `Vector`)

| Python method | Status | Note |
|---|---|---|
| `__init__` | ✅ | Inline string |
| `numberPaths` | ✅ | `DocClassMethod(numberPaths)` |
| `pathAtRank` | ✅ | `DocClassMethod(pathAtRank)` |
| `rankAtParam` | ✅ | `DocClassMethod(rankAtParam)` |
| `appendPath` | ✅ | `DocClassMethod(appendPath)` |
| `concatenate` | ✅ | `DocClassMethod(concatenate)` |
| `flatten` | ✅ | `DocClassMethod(flatten)` |

---

## path/spline.cc

### SplineB1, SplineB3

| Python method | Status | Note |
|---|---|---|
| All methods | ❌ No C++ doc | `PYHPP_DEFINE_METHOD` — no `///` in spline headers |

---

## problem_target/goal-configurations.cc

### GoalConfigurations

| Python method | Status | Note |
|---|---|---|
| `computePath` | ❌ No C++ doc | No `///` in `goal-configurations.hh` |
| `reached` | ❌ No C++ doc | Same |

---

## path_optimization/spline-gradient-based-abstract.cc

### SplineGradientBasedAbstractB1/B3, LinearConstraint, QuadraticProgram

| Python method / property | Status | Note |
|---|---|---|
| All methods and properties | ❌ No C++ doc | No `///` in the corresponding headers |
