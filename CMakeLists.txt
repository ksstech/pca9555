idf_component_register(	SRCS pca9555.c 
						INCLUDE_DIRS . 
						REQUIRES hal_esp32
						PRIV_REQUIRES buffers endpoints syslog printf common systiming hal_esp32
						)
