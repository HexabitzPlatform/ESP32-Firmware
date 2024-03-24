idf_component_register(SRCS "Station_WiFi_AP_Driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "nvs_flash main" 
                    PRIV_REQUIRES "nvs_flash")

