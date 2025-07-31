if(CONFIG_STRIM_BUS485)
    zephyr_library()

    zephyr_library_sources_ifdef(CONFIG_STRIM_BUS485 src/bus485.c)

    zephyr_include_directories(src)
endif()