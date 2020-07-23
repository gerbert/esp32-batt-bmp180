#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp32-batt-bmp180

COMPONENT_ADD_INCLUDEDIRS := include

include $(IDF_PATH)/make/project.mk

doc: all
	doxygen Doxyfile.in

