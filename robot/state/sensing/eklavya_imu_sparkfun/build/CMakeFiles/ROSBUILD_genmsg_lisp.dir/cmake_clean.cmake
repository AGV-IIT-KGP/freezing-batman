FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/eklavya_imu_sparkfun/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/RazorImu.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_RazorImu.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
