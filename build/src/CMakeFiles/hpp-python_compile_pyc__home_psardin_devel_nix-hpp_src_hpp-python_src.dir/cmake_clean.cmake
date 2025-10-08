file(REMOVE_RECURSE "pyhpp/__init__.pyc"
     "pyhpp/manipulation/constraint_graph_factory.pyc")

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(
    CMakeFiles/hpp-python_compile_pyc__home_psardin_devel_nix-hpp_src_hpp-python_src.dir/cmake_clean_${lang}.cmake
    OPTIONAL)
endforeach()
