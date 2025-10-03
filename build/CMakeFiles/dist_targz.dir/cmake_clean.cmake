file(REMOVE_RECURSE
  "doc/doxygen-html"
  "doc/doxygen.log"
  "doc/hpp-python.doxytag"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/dist_targz.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
