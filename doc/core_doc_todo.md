# pyhpp.core — documentation status

## Legend

- ✅ **Documented** — docstring present in the binding
- ❌ **No C++ doc** — C++ method has no `///` → nothing to add
- ⚠️ **Python-only** — no direct C++ equivalent → wrapper or modified signature
- ⚠️ **Signature diff.** — Python wrapper hides output params → `DocClassMethod` would trigger a Boost.Python static_assert

---

## path.cc

### Path

| Python method | Status | Note |
|---|---|---|
| `__call__` (×2) | ⚠️ Python-only | Wrappers returning `(q, success)` and `success` |
| `eval` (×2) | ⚠️ Python-only | Same |
| `derivative` | ⚠️ Python-only | Wrapper returning a numpy vector |
| `constraints` | ❌ No C++ doc | No `///` on `constraints()` in `path.hh` |
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
| `StraightPath.__init__` | ⚠️ Python-only | Factory wrapper for `StraightPath::create` |

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
| `steeringMethod` (getter/setter) | ❌ No C++ doc | Python wrapper struct; no `///` in `problem.hh` |
| `configValidation` (getter/setter) | ❌ No C++ doc | Same |
| `pathValidation` (getter/setter) | ❌ No C++ doc | Same |
| `pathProjector` (getter/setter) | ❌ No C++ doc | Same |
| `distance` (getter/setter) | ❌ No C++ doc | Same |
| `target` (getter/setter) | ❌ No C++ doc | Same |
| `configurationShooter` (getter/setter) | ❌ No C++ doc | Same |
| `errorThreshold` (def_readwrite) | ⚠️ Python-only | Python wrapper field — default threshold for ConfigProjector creation |
| `maxIterProjection` (def_readwrite) | ⚠️ Python-only | Same |
| `addPartialCom` | ⚠️ Python-only | |
| `getPartialCom` | ⚠️ Python-only | |
| `createRelativeComConstraint` | ⚠️ Python-only | |
| `createTransformationConstraint` (×2) | ⚠️ Python-only | |
| `setConstantRightHandSide` | ⚠️ Python-only | |
| `applyConstraints` | ⚠️ Python-only | |
| `isConfigValid` | ⚠️ Python-only | |
| `setConstraints` | ⚠️ Python-only | |
| `getConstraints` | ⚠️ Python-only | |
| `setRightHandSideFromConfig` | ⚠️ Python-only | |
| `addNumericalConstraintsToConfigProjector` (×2) | ⚠️ Python-only | |
| `createComBetweenFeet` | ⚠️ Python-only | |
| `directPath` | ⚠️ Signature diff. | Returns `(valid, path, report)`; hides output params |

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
| `addNode` | ⚠️ Python-only | Wrapper on `roadmap.addNode(config)` |
| `nearestNode` (×4) | ⚠️ Python-only | Return `(config, minDistance)` |
| `nearestNodes` (×2) | ⚠️ Python-only | |
| `addNodeAndEdges` | ⚠️ Python-only | |
| `addNodeAndEdge` | ⚠️ Python-only | |
| `addEdge` (×2) | ⚠️ Python-only | |
| `nodes` | ⚠️ Python-only | Returns list of configurations |
| `nodesConnectedComponent` | ⚠️ Python-only | |
| `initNode` (×2) | ⚠️ Python-only | |
| `connectedComponents` | ⚠️ Python-only | |
| `numberConnectedComponents` | ⚠️ Python-only | |
| `getConnectedComponent` | ⚠️ Python-only | |
| `connectedComponentOfNode` | ⚠️ Python-only | |

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
| `maxIterations` (setter) | ✅ | `DOC_PP_MAXITER_SET` (getter/setter pair → `DocClassMethod` unsafe on getter) |
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
| `safety` | ❌ No C++ doc | `def_readwrite`; no `///` in `simple-time-parameterization.hh` |
| `order` | ❌ No C++ doc | Same |
| `maxAcceleration` | ❌ No C++ doc | Same |

### SplineGradientBased (×3 templates)

| Python property | Status | Note |
|---|---|---|
| `alphaInit`, `alwaysStopAtFirst`, `costOrder`, etc. | ❌ No C++ doc | `def_readwrite` on public fields without `///` |

---

## steering-method.cc

### SteeringMethod

| Python method | Status | Note |
|---|---|---|
| `steer` | ✅ | `DocClassMethod(steer)` |
| `problem` | ✅ | `DocClassMethod(problem)` |
| `constraints` (setter) | ✅ | `DocClassMethod(constraints)` |
| `constraints` (getter) | ✅ | `"Get constraint set."` inline (getter/setter pair → `DocClassMethod` unsafe on getter) |
| `__call__` | ⚠️ Python-only | Wrapper on `operator()` |

---

## node.cc

### Node

| Python method | Status | Note |
|---|---|---|
| `addOutEdge` | ✅ | `DocClassMethod(addOutEdge)` |
| `addInEdge` | ✅ | `DocClassMethod(addInEdge)` |
| `connectedComponent` (getter) | ❌ No C++ doc | No `///` on the getter in `node.hh` |
| `connectedComponent` (setter) | ✅ | `"Store the connected component the node belongs to."` inline |
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
| `apply` | ⚠️ Signature diff. | Wrapper modifying `q` in-place |
| `isSatisfied` (×2) | ⚠️ Signature diff. | Wrappers modifying `error` in-place |
| `copy` | ⚠️ Python-only | Static wrapper |

### ConstraintSet

| Python method | Status | Note |
|---|---|---|
| `addConstraint` | ✅ | `DocClassMethod(addConstraint)` |
| `configProjector` | ✅ | `DocClassMethod(configProjector)` |

### ConfigProjector

| Method / property | Status | Note |
|---|---|---|
| `solver` | ✅ | `DocClassMethod(solver)` |
| `lineSearchType` (add_property) | ✅ | `DOC_CP_LINESEARCHTYPE` from `config-projector.hh` |
| `add` | ✅ | `DocClassMethod(add)` |
| `lastIsOptional` (getter/setter) | ❌ No C++ doc | No `///` in `config-projector.hh` |
| `maxIterations` (getter) | ✅ | `DOC_CP_MAXITER_GET` |
| `maxIterations` (setter) | ✅ | `DOC_CP_MAXITER_SET` |
| `errorThreshold` (getter) | ✅ | `DOC_CP_ERRTHRESH_GET` |
| `errorThreshold` (setter) | ✅ | `DOC_CP_ERRTHRESH_SET` |
| `residualError` | ✅ | `DocClassMethod(residualError)` |
| `sigma` | ✅ | `DocClassMethod(sigma)` |
| `setRightHandSideFromConfig` | ⚠️ Python-only | |
| `setRightHandSideOfConstraint` | ⚠️ Python-only | |
| `numericalConstraints` | ⚠️ Python-only | Returns a Python list |

---

## distance.cc

### Distance

| Python method | Status | Note |
|---|---|---|
| `compute` | ✅ | `DocClassMethod(compute)` |

### WeighedDistance

| Python method | Status | Note |
|---|---|---|
| `asDistancePtr_t` | ⚠️ Python-only | |
| `getWeights` | ⚠️ Python-only | Wrapper on `dist->weights()` |
| `setWeights` | ⚠️ Python-only | Wrapper on `dist->weights(weights)` |

---

## connected-component.cc

### ConnectedComponent

| Python method | Status | Note |
|---|---|---|
| `nodes` | ✅ | `DocClassMethod(nodes)` (note: Python wrapper returns configurations, not node objects) |
| `reachableFrom` | ✅ | `DocClassMethod(reachableFrom)` |
| `reachableTo` | ✅ | `DocClassMethod(reachableTo)` |
| `__eq__` | ⚠️ Python-only | Raw pointer comparison |

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
| `validate` (py tuple) | ⚠️ Signature diff. | Returns `(bool, report)` |

### ConfigValidations

| Python method | Status | Note |
|---|---|---|
| `add` | ✅ | `DocClassMethod(add)` via `PYHPP_DEFINE_METHOD2` |
| `numberConfigValidations` | ✅ | `DocClassMethod(numberConfigValidations)` |
| `clear` | ❌ No C++ doc | No `///` in `config-validations.hh` |

---

## parameter.cc

### Parameter

| Python method | Status | Note |
|---|---|---|
| `boolValue`, `intValue`, `floatValue`, `stringValue`, `vectorValue`, `matrixValue` | ❌ No C++ doc | `PYHPP_DEFINE_METHOD` without docstring; no `///` in `parameter.hh` |
| `value` | ⚠️ Python-only | Converts to Python object by type |
| `create_bool` | ⚠️ Python-only | Factory for `bool` |

---

## path-projector.cc

### PathProjector

| Python method | Status | Note |
|---|---|---|
| `apply` | ✅ | `DocClassMethod(apply)` |
| `apply` (py tuple) | ⚠️ Signature diff. | Returns `(bool, projPath)` |
| Factories (NoneProjector, ProgressiveProjector, etc.) | ⚠️ Python-only | |

---

## path-validation.cc

### PathValidation

| Python method | Status | Note |
|---|---|---|
| `validate` | ✅ | `DocClassMethod(validate)` |
| `validate` (py tuple) | ⚠️ Signature diff. | Returns `(bool, validPart, report)` |
| `validateConfiguration` | ⚠️ Python-only | Returns `(bool, report)` |
| Factories (Discretized, Progressive, Dichotomy, etc.) | ⚠️ Python-only | |

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
| All methods | ❌ No C++ doc | `PYHPP_DEFINE_METHOD` without docstring |

---

## problem_target/goal-configurations.cc

### GoalConfigurations

| Python method | Status | Note |
|---|---|---|
| `computePath` | ❌ No C++ doc | `PYHPP_DEFINE_METHOD` without docstring |
| `reached` | ❌ No C++ doc | Same |

---

## path_optimization/spline-gradient-based-abstract.cc

### SplineGradientBasedAbstractB1/B3, LinearConstraint, QuadraticProgram

| Python method / property | Status | Note |
|---|---|---|
| All methods and properties | ❌ No C++ doc | `PYHPP_DEFINE_METHOD` or `add_property`/`def_readwrite` without `///` |
