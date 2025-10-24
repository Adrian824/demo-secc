idf_component_register(
    SRCS
        "plc_mac.c"
        "plc_ll_spi.c"
        "plc_phy_dummy.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_eth
        esp_netif
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE PLC_RX_TASK_CREATE_IN_INIT=1)
target_compile_definitions(${COMPONENT_LIB} PRIVATE PLC_FORCE_STARTED_IN_INIT=1)