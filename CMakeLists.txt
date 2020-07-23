idf_component_register(
	SRCS "pca9555.c"
	INCLUDE_DIRS "."
	PRIV_INCLUDE_DIRS
	REQUIRES hal_esp32
	PRIV_REQUIRES printf systiming
)
