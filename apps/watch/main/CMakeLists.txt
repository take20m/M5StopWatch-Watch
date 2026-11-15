# キャラクター画像（tools/make_character.py で作る。著作物なので git に入れない）があれば埋め込む。
set(watch_embed "")
set(watch_defs "")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/assets/character.bin")
    list(APPEND watch_embed "assets/character.bin")
    list(APPEND watch_defs "WATCH_HAS_CHARACTER=1")
endif()
# タップしたときの声（tools/make_voices.py で作る。これも git に入れない）
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/assets/voices.bin")
    list(APPEND watch_embed "assets/voices.bin")
    list(APPEND watch_defs "WATCH_HAS_VOICES=1")
endif()

idf_component_register(
    SRCS "main.cpp" "watch_panel.cpp" "time_sync.cpp" "watch_audio.cpp"
    INCLUDE_DIRS "."
    EMBED_FILES ${watch_embed}
    REQUIRES esp_driver_gpio esp_driver_usb_serial_jtag freertos log i2c_bus m5sw_board m5sw_display m5sw_power
             m5sw_wakeup m5sw_test_support esp_hw_support esp_timer esp_lcd esp_driver_spi esp_driver_i2s esp_wifi esp_netif esp_event nvs_flash lwip
)
if(watch_defs)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${watch_defs})
endif()
