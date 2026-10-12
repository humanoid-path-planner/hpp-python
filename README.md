# hpp-python

[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/hpp-python/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/hpp-python/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/hpp-python/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-python/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Python bindings of the
[Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc/) C++
libraries, built with Boost.Python.

Package `pyhpp` follows the C++ libraries:

| Module | Content |
| ------ | ------- |
| `pyhpp.pinocchio` | robot model (`Device`), grippers, Lie groups (hpp-pinocchio) |
| `pyhpp.constraints` | differentiable functions, implicit and explicit constraints, solvers (hpp-constraints) |
| `pyhpp.core` | problem, path planners, path optimizers, path validation, steering methods (hpp-core) |
| `pyhpp.manipulation` | manipulation device, constraint graph, manipulation planners, URDF/SRDF loading (hpp-manipulation, hpp-manipulation-urdf) |
| `pyhpp.tools` | Python helpers: xacro, constraint errors |

The [tutorials](https://humanoid-path-planner.github.io/hpp-doc/reference/hpp-tutorial/)
show how to use these bindings, and the
[Python API reference](https://humanoid-path-planner.github.io/hpp-doc/reference/hpp-python/api/)
lists their classes and methods.

## Installation

hpp-python is installed with the other HPP packages, see the
[installation instructions](https://humanoid-path-planner.github.io/hpp-doc/installation/installation.html).
To build it alone:

```bash
cmake -B build -DCMAKE_INSTALL_PREFIX=<prefix> -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --build build -t test
cmake --build build -t install
```

Dependencies: Python 3 with numpy, Boost.Python, eigenpy, pinocchio, hpp-util,
hpp-pinocchio, hpp-constraints, hpp-core, hpp-manipulation and
hpp-manipulation-urdf. The tests also need example-robot-data and
hpp-environments.

## Documentation

- Docstrings: `doc/configure.py` generates the docstrings of the bindings from
  the Doxygen XML documentation of the C++ libraries, which must therefore be
  installed (`GENERATE_XML = YES` in their Doxygen configuration).
- Stubs: with `GENERATE_PYTHON_STUBS` (default `ON`, requires
  `pybind11-stubgen`), `.pyi` stubs are generated, their signatures are
  recovered from the docstrings by `doc/fix_stubs`, and they are installed with
  the modules.
- API reference: `doc/gen_api_md.py` generates the pages of the Python API
  reference from the installed stubs. The CI generates them on each push to
  `devel`.

```bash
python doc/gen_api_md.py --stubs <prefix>/lib/python3.X/site-packages --output api
```

## License

BSD 2-Clause, see [LICENSE](LICENSE).
