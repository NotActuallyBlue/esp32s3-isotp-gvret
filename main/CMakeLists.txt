idf_component_register(SRCS "main.c" "isotp_bridge.c" "mode_mgr.c" "gvret.c" "isotp.c" "ble_server.c" "twai.c" "persist.c" "isotp_link_containers.c" "eeprom.c" "uart.c" "connection_handler.c" "display.c" "flashlog.c" "bench_sim.c" "canstats.c" "ota_update.c"
                       INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_TARGET} PRIVATE "-Wno-error" "-fcommon")