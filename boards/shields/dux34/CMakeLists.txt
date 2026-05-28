# boards/shields/dux34/CMakeLists.txt
#
# 将 dux34 盾板专属 C 源文件加入编译目标
# 仅在选中该盾板时编译（SHIELD_DUX34 由 Kconfig.shield 定义）

if(CONFIG_SHIELD_DUX34)
    # 电池状态 RGB LED 指示模块（WS2812 on D1 / P0.06）
    target_sources(app PRIVATE
        battery_rgb_indicator.c
    )
endif()
