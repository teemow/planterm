# ESP-IDF component wrapper for the planterm library (header-only; the
# headers live in ../src, the repo's PlatformIO library layout). The
# plan_bridge ESPHome component registers this directory as a local IDF
# component (add_idf_component path=<repo>/planterm) -- it replaced the
# PlatformIO-only `symlink://` library URI that ESPHome 2026.7's native
# esp-idf toolchain no longer supports. This directory must be named
# "planterm": the IDF component manager derives a local path dependency's
# component name from the directory basename, and the repo checkout dir
# itself has an unpredictable (hashed) name under ESPHome's
# external_components cache.
idf_component_register(INCLUDE_DIRS ../src)
