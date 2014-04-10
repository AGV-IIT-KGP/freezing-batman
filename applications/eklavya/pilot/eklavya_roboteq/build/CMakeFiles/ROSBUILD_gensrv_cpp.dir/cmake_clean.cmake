FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/eklavya_roboteq/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/eklavya_roboteq/SetSpeed.h"
  "../srv_gen/cpp/include/eklavya_roboteq/GetSpeed.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
