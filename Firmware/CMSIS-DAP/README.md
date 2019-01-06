# STM32F103CBT6_CMSIS-DAP_SWO
-----------------------------

Based x893's code on: https://github.com/x893/CMSIS-DAP
Based RadioOperator's code 

RadioOperator contribution is:

1. Upgrade CMSIS-DAP version to V2.00 .
2. Enable SWO_UART function(USART1), no SWO_STREAM/SWO_MANCHESTER mode.
3. CDC function improved(USART2).
4. Added a Soft-Reset function for Cortex-M.
5. Added BluePill board support, Remapped or unRemap (refer to Docs).
6. Added STLINK_V2A board support (refer to Docs).
7. Minor changes, e.g. LED handling, project files re-group......

Thanks.


*****

HavenXie contribution is:
1. LED displays different status in different modens.
2. Add PWM Mode for LED.
3. Add code that supports bootloader, which can be opened by a define.

*****

What's next?

1. To be a bootloader, can upgrading this CMSIS-DAP firmware with DFU mode.
2. Let the bootloader use the AES symmetric encryotion algorithm.
3. Make this bootloader compatible with stlinkv2.1 and jlink-ob.
4. Make an upper-computer command line to switch CMSIS-DAP/ST-LinkV2.1/JLink-OB firmware.

last update at : 2019/01/06