idf_component_register(SRCS "bsp_motor_iic.c"
                       INCLUDE_DIRS "."
                       REQUIRES i2c_module)