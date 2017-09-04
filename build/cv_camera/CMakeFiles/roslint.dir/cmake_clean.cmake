FILE(REMOVE_RECURSE
  "CMakeFiles/roslint"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/roslint.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
