# Source list.
#
# Always-compiled core: avb.c, avbutils.c, avbnet.c, mrp.c, avbstats.c.
#   - mrp.c holds the MRP family (MSRP / MVRP / MAAP) — the generic
#     MRP state machines (Applicant/Registrar/LeaveAll/timers per
#     IEEE 802.1Q-2018 §10) plus the MSRP and MVRP application logic
#     and the bridge-only admission control. Used by both endpoint
#     and bridge builds. Renamed from msrp.c in the MRP-refactor.
#
# Endpoint-only:
#   - avtp.c (now AVTPDU stream construction + talker/listener tasks
#     after the SRP move). Excluded from the bridge build because the
#     bridge has no talker/listener.
#   - avbcodec.c, avbpll.c — bridge has no codec or media-clock PLL.
#   - avbsrc.c, the sample-rate converter for codecs without an APLL.
#
# Bridge-only:
#   - avbbridge.c (L2 forwarding + codec/PLL no-op stubs)
#   - avbfqtss.c (802.1Qav credit-based shaper)
#
# Gated by CONFIG_ESP_AVB_ATDECC (defaults n on bridge, y on endpoint):
#   - atdecc.c (IEEE 1722.1 ADP/AECP/ACMP). avb.c's AVTP RX
#     dispatcher #ifdef-routes the ATDECC subtypes through it.
#   - atdecc_unsol.c (registered controllers and unsolicited
#     notifications).

set(avb_srcs avb.c avbutils.c avbnet.c mrp.c avbstats.c)
set(avb_embed "")

if(CONFIG_ESP_AVB_ROLE_BRIDGE)
    list(APPEND avb_srcs avbbridge.c avbfqtss.c)
else()
    list(APPEND avb_srcs avtp.c avbcodec.c avbpll.c avbsrc.c)
endif()

if(CONFIG_ESP_AVB_ATDECC)
    list(APPEND avb_srcs atdecc.c atdecc_unsol.c)
    list(APPEND avb_embed logo.png)
endif()

idf_component_register(SRCS ${avb_srcs}
                       REQUIRES esp_eth
                       PRIV_REQUIRES esp_app_format esp_codec_dev esp_driver_gptimer esp_driver_i2c esp_driver_i2s esp_driver_gpio esp_hal_clock esp_hw_support esp_netif esp_timer esp_wifi nvs_flash
                       INCLUDE_DIRS "." "./include"
                       EMBED_FILES ${avb_embed})

target_sources(${COMPONENT_LIB} PRIVATE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
# The converter runs per sample on a single-core station: at -Og each
# 32 x 32 multiply became a sequence of partial products and spills, and
# -O3 unrolls its 16-tap loops.
set_source_files_properties(avbsrc.c PROPERTIES COMPILE_OPTIONS "-O3")

# The release version from the manifest, reported in the ENTITY
# descriptor's firmware_version.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_LIST_DIR}/idf_component.yml")
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/idf_component.yml" avb_version_line
     REGEX "^version:")
string(REGEX REPLACE "^version: *\"?([^\"]*)\"?.*$" "\\1" avb_version
       "${avb_version_line}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
                           ESP_AVB_VERSION="${avb_version}")
