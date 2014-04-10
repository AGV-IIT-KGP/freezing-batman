FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/eklavya_roboteq/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/eklavya_roboteq/srv/__init__.py"
  "../src/eklavya_roboteq/srv/_SetSpeed.py"
  "../src/eklavya_roboteq/srv/_GetSpeed.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
