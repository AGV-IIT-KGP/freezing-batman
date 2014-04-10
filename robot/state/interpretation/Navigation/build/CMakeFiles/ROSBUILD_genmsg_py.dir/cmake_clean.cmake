FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/Navigation/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/Navigation/msg/__init__.py"
  "../src/Navigation/msg/_RazorImu.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
