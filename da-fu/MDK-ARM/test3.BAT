SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\dotnet\;C:\Program Files\Git\cmd;C:\Program Files\MATLAB\R2024a\runtime\win64;C:\Program Files\MATLAB\R2024a\bin;C:\Program Files (x86)\Microsoft SQL Server\160\Tools\Binn\;C:\Program Files\Microsoft SQL Server\160\Tools\Binn\;C:\Program Files\Microsoft SQL Server\Client SDK\ODBC\170\Tools\Binn\;C:\Program Files\Microsoft SQL Server\160\DTS\Binn\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Users\37721\PROG\mingw64\bin;C:\Users\37721\PROG\ucrt64\bin;C:\Users\37721\PROG\usr\bin;C:\Users\37721\PROG\clang64\bin;C:\Program Files (x86)\Windows Kits\10\Windows Performance Toolkit\;C:\Users\37721\AppData\Local\Microsoft\WindowsApps;C:\Users\37721\PROJECT\Microsoft VS Code\bin;C:\Users\37721\Desktop\MinGW\bin;C:\Users\37721\Desktop\gcc-arm-none-eabi-10.3-2021.10-win32\Arm-GNU-Toolchian\bin;C:\Users\37721\Downloads\CLion 2024.3.5\bin;;C:\Users\37721\PROG\mingw64;C:\Users\37721\PROG\mingw64\bin\make.exe;C:\Users\37721\PROG\mingw64\bin\g++.exe;
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F103RC
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=test3
SET CPU_CLOCK=0x007A1200
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "test3\startup_stm32f103xe._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\rtc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_msp.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\ws2812.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\bsp_dwt.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\ctr.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\vibra.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_gpio_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_rtc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_rtc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_rcc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_cortex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_pwr.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_flash.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_flash_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_exti.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\stm32f1xx_hal_tim_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "test3\system_stm32f1xx.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "test3\test3.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "test3\test3.axf" --i32combined --output "test3\test3.hex"
