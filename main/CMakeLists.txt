idf_component_register(SRCS "app_main.cpp"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "data/module_drivers.json"
                       REQUIRES esp_event esp_http_server esp_netif)
