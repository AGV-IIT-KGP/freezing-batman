FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/eklavya_roboteq/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/SetSpeed.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetSpeed.lisp"
  "../srv_gen/lisp/GetSpeed.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetSpeed.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
