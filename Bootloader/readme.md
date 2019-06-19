这两个都是ST-LinkV21的bootloader
区别：
Bootloader-V2.1.bin是纯bootloader 没有版本号和序列码
Bootloader-V2.2.bin包括bootloader和版本号或者序列码

V4.3.0及其以前版本的STM32 ST-LINK Utility可以识别Bootloader-V2.1.bin
V4.5.0版本的STM32 ST-LINK Utility不再识别Bootloader-V2.1.bin，只能识别Bootloader-V2.2.bin
