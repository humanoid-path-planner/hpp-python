# pyhpp.constraints — documentation status

## Legend

- ✅ **Documented** — docstring present in the binding
- ❌ **No C++ doc** — C++ method has no `///` → nothing to add
- ⚠️ **Python-only** — no direct C++ equivalent → doc written in binding or to be written
- ⚠️ **Signature diff.** — Python wrapper hides output params → `DocClassMethod` would trigger a Boost.Python static_assert

---

## differentiable-function.cc

### DifferentiableFunction

| Python method / property | Status | Note |
|---|---|---|
| `__call__` | ✅ | `DOC_DF_CALL` from `operator()` in `differentiable-function.hh` |
| `J` | ✅ | Inline string (Python-only: returns Jacobian as numpy array) |
| `name` | ✅ | `DocClassMethod(name)` |
| `ni` | ✅ | `DOC_DF_NI` from `inputSize()` in `differentiable-function.hh` |
| `no` | ✅ | `DOC_DF_NO` from `outputSize()` |
| `ndi` | ✅ | `DOC_DF_NDI` from `inputDerivativeSize()` |
| `ndo` | ✅ | `DOC_DF_NDO` from `outputDerivativeSize()` |
| `value` | ✅ | `DocClassMethod(value)` |
| `jacobian` | ✅ | `DocClassMethod(jacobian)` |
| `outputSpace` | ✅ | `DocClassMethod(outputSpace)` |
| `inputSize` | ✅ | `DocClassMethod(inputSize)` |
| `outputSize` | ✅ | `DocClassMethod(outputSize)` |
| `inputDerivativeSize` | ✅ | `DocClassMethod(inputDerivativeSize)` |
| `outputDerivativeSize` | ✅ | `DocClassMethod(outputDerivativeSize)` |
| `impl_compute` | ❌ No C++ doc | Protected pure virtual, no `///` in the public interface |
| `impl_jacobian` | ❌ No C++ doc | Same |

### Manipulability / MinManipulability

| Python method | Status | Note |
|---|---|---|
| `Manipulability.__init__` | ✅ | Inline string — factory `Manipulability::create` |
| `Manipulability.lockJoint` | ✅ | Inline string — no `///` in `manipulability.hh` |
| `MinManipulability.__init__` | ✅ | Inline string — factory `MinManipulability::create` |
| `MinManipulability.lockJoint` | ✅ | Inline string — no `///` in `manipulability.hh` |

---

## implicit.cc

### Implicit

| Python method | Status | Note |
|---|---|---|
| `__init__` | ✅ | Inline string — wrapper for `Implicit::create` |
| `comparisonType` (getter) | ✅ | `DOC_COMPARISONTYPE_GET` (`DocClassMethod` unsafe: setter has `comp` param → kwargs mismatch) |
| `comparisonType` (setter) | ✅ | `DOC_COMPARISONTYPE_SET` |
| `function` | ✅ | `DocClassMethod(function)` |
| `parameterSize` | ✅ | `DocClassMethod(parameterSize)` |
| `rightHandSideSize` | ✅ | `DocClassMethod(rightHandSideSize)` |
| `getFunctionOutputSize` | ✅ | Inline string — static method, no direct C++ equivalent |

---

## explicit.cc

| Python method | Status | Note |
|---|---|---|
| `createExplicit` | ✅ | Inline string — wrapper around `Explicit::create` |

---

## explicit-constraint-set.cc

### ExplicitConstraintSet

| Python method | Status | Note |
|---|---|---|
| `add` | ✅ | `DocClassMethod(add)` |
| `errorSize` | ✅ | `DocClassMethod(errorSize)` |

---

## iterative-solver.cc

### HierarchicalIterative

| Method / property | Status | Note |
|---|---|---|
| `__str__` | ⚠️ Python-only | Uses `to_str<HierarchicalIterative>` |
| `add` | ✅ | `DocClassMethod(add)` |
| `errorThreshold` (1st add_property) | ✅ | `DOC_HI_ERRORTHRESHOLD` from `hierarchical-iterative.hh` |
| `errorThreshold` (2nd add_property, duplicate) | ❌ | Pre-existing duplicate in the code, no docstring |
| `rightHandSideFromConfig` (config only) | ✅ | `DOC_HI_RHSFC1` from `hierarchical-iterative.hh` |
| `rightHandSideFromConfig` (constraint + config) | ✅ | `DOC_HI_RHSFC2` |
| `rightHandSide` (setter constraint + rhs) | ✅ | `DOC_HI_RHS_SET1` |
| `rightHandSide` (setter rhs only) | ✅ | `DOC_HI_RHS_SET2` |
| `rightHandSide` (getter) | ✅ | `DOC_HI_RHS_GET` |
| `maxIterations` (add_property) | ✅ | `DOC_HI_MAXITERATIONS` from `hierarchical-iterative.hh` |
| `lastIsOptional` (add_property) | ✅ | Inline string — no `///` in `hierarchical-iterative.hh` (uses `//`) |
| `solveLevelByLevel` (add_property) | ✅ | Inline string — same (non-doxygen `//`) |
| `numberStacks` | ✅ | Inline string — no `///` |
| `constraintsForPriority` | ✅ | Inline string — wrapper returning a Python list |
| `dimension` | ✅ | Inline string — no `///` on the exposed method |

---

## by-substitution.cc

### BySubstitution

| Method / property | Status | Note |
|---|---|---|
| `SolverStatus` (enum) | ❌ No C++ doc | Enum without class-level `///` |
| `explicitConstraintSetHasChanged` | ✅ | `DocClassMethod(explicitConstraintSetHasChanged)` |
| `solve` | ✅ | Inline string — returns `(qout, status)` tuple |
| `explicitConstraintSet` | ✅ | `DocClassMethod(explicitConstraintSet)` |
| `rightHandSideFromConfig` (config only) | ✅ | `DOC_BS_RHSFC1` from `by-substitution.hh` |
| `rightHandSideFromConfig` (constraint + config) | ✅ | `DOC_BS_RHSFC2` |
| `rightHandSide` (setter constraint + rhs) | ✅ | `DOC_BS_RHS_SET1` |
| `rightHandSide` (setter rhs only) | ✅ | `DOC_BS_RHS_SET2` |
| `rightHandSide` (getter) | ✅ | `DOC_BS_RHS_GET` |
| `errorThreshold` (add_property) | ✅ | `DOC_BS_ERRORTHRESHOLD` from `by-substitution.hh` |
| `describeError` | ✅ | Inline string — returns list of `(constraint_name, error_norm)` pairs |

---

## generic-transformation.cc

| Python class | Status | Note |
|---|---|---|
| `Position.__init__` | ✅ | Inline string (absolute generic transformation) |
| `Orientation.__init__` | ✅ | Inline string |
| `Transformation.__init__` | ✅ | Inline string |
| `RelativePosition.__init__` | ✅ | Inline string (relative generic transformation) |
| `RelativeOrientation.__init__` | ✅ | Inline string |
| `RelativeTransformation.__init__` | ✅ | Inline string |
| `RelativeTransformationR3xSO3.__init__` | ✅ | Inline string |

---

## locked-joint.cc

### LockedJoint

| Python method | Status | Note |
|---|---|---|
| `__init__` (joint + config) | ✅ | Inline string — `createLockedJoint` wrapper |
| `__init__` (joint + config + comp) | ✅ | Inline string — `createLockedJointWithComp` wrapper |

---

## relative-com.cc

### RelativeCom

| Python method | Status | Note |
|---|---|---|
| `__init__` (create1: name+robot+joint+ref+mask) | ✅ | `DocClassMethod(create)` |
| `__init__` (create2: robot+comc+joint+ref+mask) | ⚠️ Python-only | No dedicated `///` in `relative-com.hh` |
| `__init__` (create3: name+robot+comc+joint+ref+mask) | ⚠️ Python-only | Same |
