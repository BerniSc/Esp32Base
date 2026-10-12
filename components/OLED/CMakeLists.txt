idf_component_register(SRCS "src/ssd1306Display.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "privateInclude"
                    REQUIRES I2C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
