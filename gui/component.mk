
COMPONENT_SRCDIRS +=                            \
	drivers/lcd                             \
        drivers/touch                           \
        thirdparty/gui                          \
        thirdparty/gui/lvgl                     \
        thirdparty/gui/lvgl/lv_core             \
        thirdparty/gui/lvgl/lv_draw             \
        thirdparty/gui/lvgl/lv_hal              \
        thirdparty/gui/lvgl/lv_misc             \
        thirdparty/gui/lvgl/lv_misc/lv_fonts    \
        thirdparty/gui/lvgl/lv_objx             \
        thirdparty/gui/lvgl/lv_themes           \
	tools

COMPONENT_ADD_INCLUDEDIRS := .                  \
	drivers/lcd                             \
        drivers/touch                           \
        thirdparty/gui                          \
        thirdparty/gui/lvgl                     \
        thirdparty/gui/lvgl/lv_core             \
        thirdparty/gui/lvgl/lv_draw             \
        thirdparty/gui/lvgl/lv_hal              \
        thirdparty/gui/lvgl/lv_misc             \
        thirdparty/gui/lvgl/lv_misc/lv_fonts    \
        thirdparty/gui/lvgl/lv_objx             \
        thirdparty/gui/lvgl/lv_themes           \
	tools

COMPONENT_PRIV_INCLUDEDIRS := .                 \
	drivers/lcd                             \
        drivers/touch                           \
        thirdparty/gui                          \
        thirdparty/gui/lvgl                     \
        thirdparty/gui/lvgl/lv_core             \
        thirdparty/gui/lvgl/lv_draw             \
        thirdparty/gui/lvgl/lv_hal              \
        thirdparty/gui/lvgl/lv_misc             \
        thirdparty/gui/lvgl/lv_misc/lv_fonts    \
        thirdparty/gui/lvgl/lv_objx             \
        thirdparty/gui/lvgl/lv_themes           \
	tools

COMPONENT_SUBMODULES := thirdparty/gui/lvgl
