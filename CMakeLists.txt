cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "components")

# ============================================================
# LVGL CONFIG PATH (CORRETO)
# ============================================================
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE=1)
add_compile_definitions(LV_CONF_PATH=\"lv_conf.h\")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(POSTE_INTELIGENTE_V8)