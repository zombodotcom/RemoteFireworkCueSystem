idf_component_register(
    SRCS "controller_main.cpp" "espnow_tx.cpp" "config_store.cpp" "web_server.cpp"
    INCLUDE_DIRS "."
    REQUIRES fireworkcore esp_wifi nvs_flash esp_timer log
             esp_netif esp_event freertos
             esp_http_server
    EMBED_FILES "www/index.html.gz"
)
