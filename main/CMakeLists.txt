set(SRCS "main.c" "deep_sleep.c" "wifi_connect.c" "power_log.c")

if(CONFIG_WAKE_STRATEGY_LISTEN)
    list(APPEND SRCS "strategy_listen.c")
elseif(CONFIG_WAKE_STRATEGY_ESPNOW)
    list(APPEND SRCS "strategy_espnow.c")
elseif(CONFIG_WAKE_STRATEGY_BLE)
    list(APPEND SRCS "strategy_ble.c")
elseif(CONFIG_WAKE_STRATEGY_DTIM)
    list(APPEND SRCS "strategy_dtim.c")
endif()

set(REQUIRES "")
if(CONFIG_WAKE_STRATEGY_ESPNOW)
    list(APPEND REQUIRES "esp_wifi")
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)
