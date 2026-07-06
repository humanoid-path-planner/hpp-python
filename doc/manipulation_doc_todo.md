# pyhpp.manipulation — documentation status

## Legend

- ✅ **Documented** — docstring present in the binding
- ❌ **No C++ doc** — C++ method has no `///` → `DocClassMethod` would produce an empty string
- ⚠️ **Python-only** — no direct C++ equivalent → doc to be written in the binding
- ⚠️ **Signature diff.** — Python wrapper hides output params → `DocClassMethod` would trigger a Boost.Python static_assert

---

## graph.cc

### State

| Python method | Status | Note |
|---|---|---|
| `State.name` | ✅ | `DocClassMethod(name)` |
| `State.id` | ✅ | `DocClassMethod(id)` |
| `State.configConstraint` | ❌ No C++ doc | `State::configConstraint()` declared without `///` in `state.hh` |
| `State.neighborEdges` | ✅ | `DOC_NEIGHBOREDGES` inline |

### Transition (Edge)

| Python method | Status | Note |
|---|---|---|
| `Transition.id` | ✅ | `DocClassMethod(id)` |
| `Transition.name` | ✅ | `DocClassMethod(name)` |
| `Transition.isWaypointTransition` | ✅ | `DocClassMethod(isWaypointEdge)` |
| `Transition.nbWaypoints` | ✅ | `DocClassMethod(nbWaypoints)` |
| `Transition.waypoint` | ✅ | `DocClassMethod(waypoint)` |
| `Transition.pathValidation` | ✅ | `DocClassMethod(pathValidation)` |

### Graph

| Method / property | Status | Note |
|---|---|---|
| `Graph._get_native_graph` | ⚠️ Python-only | C++ capsule for external interop |
| `Graph.robot` | ⚠️ Python-only | `def_readwrite`, no `///` |
| `Graph.maxIterations` | ❌ No C++ doc | `Graph::maxIterations()` has `///` in `graph.hh` but exposed via `PYHPP_DEFINE_GETTER_SETTER` without docstring |
| `Graph.errorThreshold` | ❌ No C++ doc | Same |
| `Graph.createState` | ✅ | `DOC_CREATESTATE` inline |
| `Graph.createTransition` | ✅ | `DOC_CREATETRANSITION` inline |
| `Graph.createWaypointTransition` | ✅ | `DOC_CREATEWAYPOINTTRANSITION` inline |
| `Graph.createLevelSetTransition` | ✅ | `DOC_CREATELEVELSETTRANSITION` inline |
| `Graph.setContainingNode` | ✅ | `DOC_SETCONTAININGNODE` inline |
| `Graph.getContainingNode` | ✅ | `DOC_GETCONTAININGNODE` inline |
| `Graph.setShort` | ✅ | `DOC_SETSHORT` inline |
| `Graph.isShort` | ✅ | `DOC_ISSHORT` inline |
| `Graph.getNodesConnectedByTransition` | ✅ | `DOC_GETNODESCONNECTEDBYTRANSITION` inline |
| `Graph.setWeight` | ✅ | `DOC_SETWEIGHT` inline |
| `Graph.getWeight` | ✅ | `DOC_GETWEIGHT` inline |
| `Graph.setWaypoint` | ✅ | `DOC_SETWAYPOINT` inline |
| `Graph.getState` *(by name)* | ⚠️ Python-only | Lookup by name in internal map |
| `Graph.getTransition` *(by name)* | ⚠️ Python-only | Lookup by name in internal map |
| `Graph.getStates` | ⚠️ Python-only | Returns list of `PyWState` |
| `Graph.getTransitions` | ⚠️ Python-only | Returns list of `PyWEdge` |
| `Graph.getStateNames` | ⚠️ Python-only | Returns list of names |
| `Graph.getTransitionNames` | ⚠️ Python-only | Returns list of names |
| `Graph.getStateFromConfiguration` | ✅ | `DOC_GETSTATE` inline |
| `Graph.addNumericalConstraint` | ✅ | `DOC_ADDNUMERICALCONSTRAINT` inline |
| `Graph.addNumericalConstraintsToState` | ✅ | `DOC_ADDNUMERICALCONSTRAINTSTOSTATE` inline |
| `Graph.addNumericalConstraintsToTransition` | ✅ | `DOC_ADDNUMERICALCONSTRAINTSTOTRANSITION` inline |
| `Graph.addNumericalConstraintsToGraph` | ⚠️ Python-only | Takes a list, no direct equivalent |
| `Graph.addNumericalConstraintsForPath` | ✅ | `DOC_ADDNUMERICALCONSTRAINTSFORPATH` inline |
| `Graph.getNumericalConstraintsForState` | ✅ | `DOC_GETNUMERICALCONSTRAINTSFORSTATE` inline |
| `Graph.getNumericalConstraintsForEdge` | ✅ | `DOC_GETNUMERICALCONSTRAINTSFOREDGE` inline |
| `Graph.getNumericalConstraintsForGraph` | ✅ | `DOC_GETNUMERICALCONSTRAINTSFORGRAPH` inline |
| `Graph.resetConstraints` | ✅ | `DOC_RESETCONSTRAINTS` inline |
| `Graph.registerConstraints` | ✅ | `DOC_REGISTERCONSTRAINTS` inline |
| `Graph.createPlacementConstraint` | ✅ | `DOC_CREATEPLACEMENTCONSTRAINT` inline |
| `Graph.createPrePlacementConstraint` | ✅ | `DOC_CREATEPREPLACEMENTCONSTRAINT` inline |
| `Graph.createGraspConstraint` | ⚠️ Python-only | Wrapper around `Handle::createGrasp`, no `///` |
| `Graph.createPreGraspConstraint` | ⚠️ Python-only | Wrapper around `Handle::createPreGrasp` |
| `Graph.getConfigErrorForState` | ✅ | `DOC_GETCONFIGERRORFORSTATE` inline |
| `Graph.getConfigErrorForTransition` | ✅ | `DOC_GETCONFIGERRORFORTRANSITION` inline |
| `Graph.getConfigErrorForTransitionLeaf` | ✅ | `DOC_GETCONFIGERRORFORTRANSITIONLEAF` inline |
| `Graph.getConfigErrorForTransitionTarget` | ✅ | `DOC_GETCONFIGERRORFORTRANSITIONTARGET` inline |
| `Graph.applyStateConstraints` | ✅ | `DOC_APPLYSTATECONSTRAINTS` inline |
| `Graph.applyLeafConstraints` | ✅ | `DOC_APPLYLEAFCONSTRAINTS` inline |
| `Graph.generateTargetConfig` | ✅ | `DOC_GENERATETARGETCONFIG` inline |
| `Graph.addLevelSetFoliation` | ✅ | `DOC_ADDLEVELSETFOLIATION` inline |
| `Graph.getSecurityMarginMatrixForTransition` | ✅ | `DOC_GETSECURITYMARGINMATRIXFORTRANSITION` inline |
| `Graph.setSecurityMarginForTransition` | ✅ | `DOC_SETSECURITYMARGINFORTRANSITION` inline |
| `Graph.getRelativeMotionMatrix` | ✅ | `DOC_GETRELATIVEMOTIONMATRIX` inline |
| `Graph.removeCollisionPairFromTransition` | ✅ | `DOC_REMOVECOLLISIONPAIRFROMTRANSITION` inline |
| `Graph.createSubGraph` | ✅ | `DOC_CREATESUBGRAPH` inline |
| `Graph.setTargetNodeList` | ✅ | `DOC_SETTARGETNODELIST` inline |
| `Graph.displayStateConstraints` | ✅ | `DOC_DISPLAYSTATECONSTRAINTS` inline |
| `Graph.displayTransitionConstraints` | ✅ | `DOC_DISPLAYTRANSITIONCONSTRAINTS` inline |
| `Graph.displayTransitionTargetConstraints` | ✅ | `DOC_DISPLAYTRANSITIONTARGETCONSTRAINTS` inline |
| `Graph.display` | ✅ | `DOC_DISPLAY` inline |
| `Graph.initialize` | ✅ | `DOC_INITIALIZE` inline |
| `Graph.transitionAtParam` | ⚠️ Python-only | Static method, wraps `Graph::edgeAtParam` |

---

## device.cc

### Handle

| Method / property | Status | Note |
|---|---|---|
| `Handle.name` | ✅ | Inline string from `handle.hh` |
| `Handle.localPosition` | ✅ | Inline string from `handle.hh` |
| `Handle.mask` | ✅ | Inline string from `handle.hh` |
| `Handle.maskComp` | ✅ | Inline string from `handle.hh` |
| `Handle.clearance` | ✅ | Inline string from `handle.hh` |
| `Handle.approachingDirection` | ✅ | Inline string from `handle.hh` |
| `Handle.createGrasp` | ✅ | `DocClassMethod(createGrasp)` |
| `Handle.createPreGrasp` | ✅ | `DocClassMethod(createPreGrasp)` |
| `Handle.createGraspComplement` | ✅ | `DocClassMethod(createGraspComplement)` |
| `Handle.createGraspAndComplement` | ✅ | `DocClassMethod(createGraspAndComplement)` |
| `Handle.getParentJointId` | ✅ | Inline string (Python-only) |

### Device

| Python method | Status | Note |
|---|---|---|
| `Device.setRobotRootPosition` | ❌ No C++ doc | Declared without `///` in `device.hh` |
| `Device.handles` | ❌ No C++ doc | Public member without `///` |
| `Device.grippers` | ❌ No C++ doc | Public member without `///` |
| `Device.getJointNames` | ⚠️ Python-only | Wrapper without direct C++ equivalent |
| `Device.getJointConfig` | ⚠️ Python-only | Wrapper without direct C++ equivalent |
| `Device.setJointBounds` | ⚠️ Python-only | No docstring in the binding |
| `Device.contactSurfaceNames` | ✅ | Inline string |
| `Device.contactSurfaces` | ✅ | Inline string |
| `Device.addHandle` | ✅ | Inline string |
| `Device.addGripper` | ✅ | Inline string |
| `Device.modelsInfo` | ✅ | Inline string |

---

## problem.cc

| Python method | Status | Note |
|---|---|---|
| `Problem.constraintGraph` (getter) | ✅ | Inline string (`DocClassMethod` unsafe: setter kwargs > getter args) |
| `Problem.constraintGraph` (setter) | ✅ | Inline string |
| `Problem.checkProblem` | ✅ | `DocClassMethod(checkProblem)` |
| `Problem.steeringMethod` (getter/setter ×3) | ❌ No C++ doc | No `///` in `manipulation::Problem` |

---

## path-planner.cc — TransitionPlanner

| Python method | Status | Note |
|---|---|---|
| `innerPlanner` (getter) | ✅ | Inline string (`DocClassMethod` unsafe: setter kwargs > getter args) |
| `innerPlanner` (setter) | ✅ | Inline string |
| `innerProblem` | ✅ | `DocClassMethod(innerProblem)` |
| `computePath` | ✅ | `DocClassMethod(computePath)` |
| `planPath` *(deprecated)* | ✅ | `DocClassMethod(planPath)` |
| `directPath` | ⚠️ Signature diff. | Hides `success` and `status` output params → kwargs count mismatch |
| `validateConfiguration` | ⚠️ Signature diff. | Hides `report` output param → kwargs count mismatch |
| `optimizePath` | ✅ | `DocClassMethod(optimizePath)` |
| `timeParameterization` | ✅ | `DocClassMethod(timeParameterization)` |
| `setEdge` *(deprecated)* | ✅ | `DocClassMethod(setEdge)` |
| `setTransition` | ✅ | `DocClassMethod(setEdge)` |
| `setReedsAndSheppSteeringMethod` | ✅ | `DocClassMethod(setReedsAndSheppSteeringMethod)` |
| `pathProjector` | ✅ | `DocClassMethod(pathProjector)` |
| `clearPathOptimizers` | ✅ | `DocClassMethod(clearPathOptimizers)` |
| `addPathOptimizer` | ✅ | `DocClassMethod(addPathOptimizer)` |

## path-planner.cc — ManipulationPlanner / StatesPathFinder

| Python class | Status | Note |
|---|---|---|
| `ManipulationPlanner` | ❌ No C++ doc | Constructor only, no class-level `///` |
| `StatesPathFinder` | ❌ No C++ doc | Same |

## path-planner.cc — EndEffectorTrajectory *(deprecated)*

| Python method | Status | Note |
|---|---|---|
| `nRandomConfig` (getter) | ✅ | Inline string (`DocClassMethod` produced empty string) |
| `nDiscreteSteps` (getter) | ✅ | Inline string |
| `checkFeasibilityOnly` (getter) | ✅ | Inline string (`DocClassMethod` unsafe: setter kwargs > getter args) |

---

## path-optimizer.cc

| Python class / method | Status | Note |
|---|---|---|
| `EnforceTransitionSemantic` | ✅ | `DOC_ENFORCE_TRANSITION_SEMANTIC` inline |
| `RandomShortcut` | ❌ No C++ doc | No class-level `///` in `random-shortcut.hh` |
| `GraphRandomShortcut` | ⚠️ Python-only | Factory function, no direct equivalent |
| `GraphPartialShortcut` | ⚠️ Python-only | Same |
| `SplineGradientBased_bezier1/3` attributes (`alphaInit`, `alwaysStopAtFirst`, `costOrder`, `usePathLengthAsWeights`, `reorderIntervals`, `linearizeAtEachStep`, `checkJointBound`, `returnOptimum`, `costThreshold`, `guessThreshold`, `QPAccuracy`) | ❌ No C++ doc | `def_readwrite` without `///` |

---

## path-projector.cc

| Python function | Status | Note |
|---|---|---|
| `NoneProjector` | ⚠️ Python-only | |
| `ProgressiveProjector` | ⚠️ Python-only | Wraps `pathProjector::Progressive::create` |
| `DichotomyProjector` | ⚠️ Python-only | Wraps `pathProjector::Dichotomy::create` |
| `GlobalProjector` | ⚠️ Python-only | Wraps `pathProjector::Global::create` |
| `RecursiveHermiteProjector` | ⚠️ Python-only | Wraps `pathProjector::RecursiveHermite::create` |

---

## steering-method.cc

| Python class / method | Status | Note |
|---|---|---|
| `GraphSteeringMethod` (no methods) | ❌ No C++ doc | No class-level `///` |
| `EndEffectorTrajectorySteeringMethod.setTrajectoryConstraint` | ✅ | Inline string from `end-effector-trajectory.hh` |
| `EndEffectorTrajectorySteeringMethod.setTrajectory` | ✅ | Inline string from `end-effector-trajectory.hh` |

---

## steering_method/cartesian.cc

| Method / property | Status | Note |
|---|---|---|
| `Cartesian.maxIterations` | ✅ | Inline string |
| `Cartesian.errorThreshold` | ✅ | Inline string |
| `Cartesian.trajectoryConstraint` | ✅ | Inline string |
| `Cartesian.nDiscreteSteps` | ✅ | Inline string |
| `Cartesian.timeRange` | ✅ | Inline string |
| `Cartesian.setRightHandSide` (×2) | ✅ | Inline string |
| `Cartesian.getRightHandSide` | ✅ | Inline string |
| `Cartesian.planPath` | ✅ | Inline string |
| `makePiecewiseLinearTrajectory` | ✅ | Inline string |

---

## urdf/util.cc

| Python function | Status | Note |
|---|---|---|
| `loadModel` | ⚠️ Python-only | Overload that also calls `srdf::loadModelFromFile` |
| `loadModelFromString` | ⚠️ Python-only | Overload with XML string |
