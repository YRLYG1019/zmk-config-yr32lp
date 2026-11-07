if(CONFIG_INPUT_PIM447)
  zephyr_library_named(input_pim447)
  zephyr_library_sources(drivers/input/input_pim447.c)
endif()
