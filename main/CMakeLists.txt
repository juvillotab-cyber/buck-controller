idf_component_register(SRCS "main.c" 
                            "wifi_manager.c" 
                            "mqtt_manager.c" 
                            "buck_pwm.c" 
                            "ctrl_3p3z.c" 
                            "buck_adc.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        driver 
                        esp_driver_mcpwm 
                        esp_adc esp_wifi
                        esp_netif
                        esp_event
                        mqtt
                        nvs_flash
                        esp-tls
)
