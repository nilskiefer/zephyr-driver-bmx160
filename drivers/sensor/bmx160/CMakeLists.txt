zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BMX160 bmx160.c)
