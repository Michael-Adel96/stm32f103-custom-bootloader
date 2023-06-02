## Usage steps
1. BL Project<br>
    a. Configure app start address in bootloader_config.h file.<br>

2. APP Project<br>
    a. configure app linker script to the new flash start address.<br>
    b. Provide the IVT offset.<br>
        i. Go to system_stm32f1xx.c file.<br>
        ii. then modify VECT_TAB_OFFSET to be the start address of the flash in APP project.<br>