idf_component_register(SRCS "main.cpp" "tests/componentTests.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES json nvs_flash Logger Wifi WebServer GPIO)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
