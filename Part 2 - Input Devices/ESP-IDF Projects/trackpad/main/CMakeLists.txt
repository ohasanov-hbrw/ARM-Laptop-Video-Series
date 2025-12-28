set(srcs "esp_hid_device_main.c")
set(include_dirs ".")


idf_component_register(	SRCS "${srcs}"
                      	INCLUDE_DIRS "${include_dirs}"
                    	REQUIRES spi_flash esp_driver_i2c bt esp_hid
                    	PRIV_REQUIRES nvs_flash)
                    	
