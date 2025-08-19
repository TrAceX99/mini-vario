idf_component_register(SRCS "bmp3.c"
                    INCLUDE_DIRS "include")

# Enable 64-bit integer compensation (returns scaled int values instead of double)
target_compile_definitions(${COMPONENT_LIB} PUBLIC BMP3_64BIT_COMPENSATION=1)
