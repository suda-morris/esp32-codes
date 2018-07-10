#
# Main Makefile. This is basically the same as a component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_SRCDIRS := . \
	lv_core \
	lv_draw \
	lv_objx \
	lv_hal \
	lv_misc \
	lv_misc/lv_fonts \
	lv_themes
	
COMPONENT_ADD_INCLUDEDIRS := $(COMPONENT_SRCDIRS) ../