FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/global_planner/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/global_planner/msg/__init__.py"
  "../src/global_planner/msg/_Seed.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
