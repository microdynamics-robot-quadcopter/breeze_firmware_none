#显示更详细的错误列表
#SET(CMAKE_VERBOSE_MAKEFILE on)
SET(PERIPH_LIB ../STM32F10x_StdPeriph_Lib_V3.5.0/)
INCLUDE_DIRECTORIES(
    ${PERIPH_LIB}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
    ${PERIPH_LIB}/Libraries/CMSIS/CM3/CoreSupport/
    ${PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/inc/
#${PERIPH_LIB}/Project/STM32F10x_StdPeriph_Template
)
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/User
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_IIC
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_PWM
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_SPI
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_Sys
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_Tim
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_USART
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_Flash
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_Delay
    ${PROJECT_SOURCE_DIR}/Drivers/Driver_EEPROM
    ${PROJECT_SOURCE_DIR}/System/System_Led
    ${PROJECT_SOURCE_DIR}/System/System_MS5611
    ${PROJECT_SOURCE_DIR}/System/System_RPData
    ${PROJECT_SOURCE_DIR}/System/System_Monitor
    ${PROJECT_SOURCE_DIR}/System/System_Battery
    ${PROJECT_SOURCE_DIR}/System/System_MPU6050
    ${PROJECT_SOURCE_DIR}/System/System_NRF24L01
    ${PROJECT_SOURCE_DIR}/Algorithm/Algorithm_BAR
    ${PROJECT_SOURCE_DIR}/Algorithm/Algorithm_IMU
    ${PROJECT_SOURCE_DIR}/Algorithm/Algorithm_Filter
    ${PROJECT_SOURCE_DIR}/Algorithm/Algorithm_Flight
    ${PROJECT_SOURCE_DIR}/Algorithm/Algorithm_Control
)

set(
    USER_SRC
    main.c
    ../Drivers/Driver_IIC/stm32f10x_driver_iic.c
    ../Drivers/Driver_PWM/stm32f10x_driver_pwm.c
    ../Drivers/Driver_SPI/stm32f10x_driver_spi.c
    ../Drivers/Driver_Sys/stm32f10x_driver_sys.c
    ../Drivers/Driver_Tim/stm32f10x_driver_tim.c
    ../Drivers/Driver_USART/stm32f10x_driver_usart.c
    ../Drivers/Driver_Delay/stm32f10x_driver_delay.c
    ../Drivers/Driver_EEPROM/stm32f10x_driver_eeprom.c
    ../System/System_Led/stm32f10x_system_led.c
    ../System/System_MS5611/stm32f10x_system_ms5611.c
    ../System/System_RPData/stm32f10x_system_rpdata.c
    ../System/System_Monitor/stm32f10x_system_monitor.c
    ../System/System_Battery/stm32f10x_system_battery.c
    ../System/System_MPU6050/stm32f10x_system_mpu6050.c
    ../System/System_NRF24L01/stm32f10x_system_nrf24l01.c
    ../Algorithm/Algorithm_BAR/stm32f10x_algorithm_bar.c
    ../Algorithm/Algorithm_IMU/stm32f10x_algorithm_imu.c
    ../Algorithm/Algorithm_Filter/stm32f10x_algorithm_filter.c
    ../Algorithm/Algorithm_Flight/stm32f10x_algorithm_flight.c
    ../Algorithm/Algorithm_Control/stm32f10x_algorithm_control.c
    ../startup_stm32f10x_md.s #is right?
)

SET(PERIPH_LIB_SRC ${PERIPH_LIB}/Libraries/STM32F10x_StdPeriph_Driver/src/)
#SET(CMAKE_VERBOSE_MAKEFILE 0)
SET(CMAKE_SYSTEM_NAME Linux)
#SET(CMAKE_BUILD_TYPE DEBUG)
SET(CMAKE_BUILD_TYPE RELEASE)
SET(CMAKE_C_COMPILER "arm-none-eabi-gcc")
SET(CMAKE_LINKER "arm-none-eabi-ld ")
SET(CMAKE_OBJCOPY "arm-none-eabi-objcopy")
SET(CMAKE_OBJDUMP "arm-none-eabi-objdump")
SET(CMAKE_SIZE "arm-none-eabi-size")
SET(LINK_SCRIPT ${CMAKE_SOURCE_DIR}/stm32f10x.ld)
#SET(FAMILY "STM32F103xB")
#SET(LINK_SCRIPT "stm32f10x.ld")
SET(FAMILY "STM32F10X_MD")
#宏 MCU 型号 FLASH大小
#STM32F10X_LD STM32F101xx STM32F102xx STM32F103xx 16 ~ 32 Kbytes
#STM32F10X_MD STM32F101xx STM32F102xx STM32F103xx 64 ~ 128 Kbytes
#STM32F10X_HD STM32F101xx STM32F103xx             256 ~ 512 Kbytes
#STM32F10X_CL STM32F105xx STM32F107xx
#startup_stm32f10x_cl.s    互联型的器件: STM32F105xx，STM32F107xx
#startup_stm32f10x_hd.s    大容量的STM32F101xx: STM32F102xx，STM32F103xx
#startup_stm32f10x_hd_vl.s 大容量的STM32F100xx:
#startup_stm32f10x_ld.s    小容量的STM32F101xx: STM32F102xx，STM32F103xx
#startup_stm32f10x_ld_vl.s 小容量的STM32F100xx
#startup_stm32f10x_md.s    中容量的STM32F101xx: STM32F102xx，STM32F103xx
#startup_stm32f10x_md_vl.s 中容量的STM32F100xx:（我项目中用的是此款芯片 stm32f100CB）
#startup_stm32f10x_xl.s    FLASH在512K到1024K字节的STM32F101xx，STM32F102xx，STM32F103xx
#（例如：像stm32f103re 这个型号的 芯片flash是512k的，启动文件用startup_stm32f10x_xl.s  或者startup_stm32f10x_hd.s都可以；）
#
#cl：互联型产品   stm32f105/107系列
#vl：超值型产品   stm32f100系列
#xl：超高密度产品 stm32f101/103系列
#ld：低密度产品   FLASH小于64K
#md：中等密度产品 FLASH=64 or 128
#hd：高密度产品   FLASH大于128

#SET(DEBUG_FLAGS "-O0 -g")
#SET(DEBUG_FLAGS "-Os -finput-charset=GB2312 -fexec-charset=GB2312 ")
SET(DEBUG_FLAGS "-Os ")

SET(MCU_FLAGS "-march=armv7-m -mcpu=cortex-m3 -mthumb")
SET(CMAKE_C_FLAGS "${MCU_FLAGS} -fno-common -fno-builtin -ffreestanding -Wall ${DEBUG_FLAGS}-O3 ") #optimization
#编译选项：
#-ffunction-sections 将函数放到自己的section中(链接时配合--gc-sections可以移除没有使用的函数)
#-fdata-sections     将data放到自己的section中(链接时配合--gc-sections可以移除没有使用的data)
#-Wl,-Map=$(IMG_PATH)/$(IMG_MAP)  生成指定的map文件
#-Wl,--gc-sections                移除唯有链接的内容
#-Wl,-T$(L_SCRIPT)                指定ld文件
#-mfloat-abi=soft    使用软件浮点运算库，链接时加入 libgcc.a
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -nostdlib -mfloat-abi=soft -ffunction-sections -fdata-sections -Wl,-Map,${PROJECT_NAME}.map -Wl,--gc-sections")
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-T${LINK_SCRIPT}")
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
add_definitions(-D${FAMILY} -DUSE_STDPERIPH_DRIVER)
set(CMAKE_EXE_LINKER_FLAGS "${DEBUG_FLAGS} -nostartfiles -T${LINK_SCRIPT}")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
LINK_DIRECTORIES(${PROJECT_BINARY_DIR})
add_executable(breeze.hex ${USER_SRC})
message(STATUS "This is c flags!!!" ${CMAKE_C_FLAGS})
