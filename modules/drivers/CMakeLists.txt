if(CONFIG_PMW3610)
  add_subdirectory(sensor/pmw3610)
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()
