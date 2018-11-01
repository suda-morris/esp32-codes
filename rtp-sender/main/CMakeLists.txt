set(COMPONENT_SRCS "rtp_example_main.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

# Embed the tf file into the final binary
set(COMPONENT_EMBED_TXTFILES ${PROJECT_PATH}/ts_files/espressif.ts)

register_component()
