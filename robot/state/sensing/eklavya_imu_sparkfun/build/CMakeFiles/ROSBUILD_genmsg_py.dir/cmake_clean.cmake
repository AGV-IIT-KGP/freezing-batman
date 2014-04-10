FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/eklavya_imu_sparkfun/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/eklavya_imu_sparkfun/msg/__init__.py"
  "../src/eklavya_imu_sparkfun/msg/_RazorImu.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
