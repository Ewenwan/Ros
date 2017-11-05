FILE(REMOVE_RECURSE
  "CMakeFiles/sphinx-doc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/sphinx-doc.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
