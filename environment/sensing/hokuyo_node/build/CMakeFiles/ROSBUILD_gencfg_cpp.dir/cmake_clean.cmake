FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/hokuyo_node/HokuyoConfig.h"
  "../docs/HokuyoConfig.dox"
  "../docs/HokuyoConfig-usage.dox"
  "../src/hokuyo_node/cfg/HokuyoConfig.py"
  "../docs/HokuyoConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
