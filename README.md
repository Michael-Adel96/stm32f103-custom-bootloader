## Usage steps
1. BL Project
    a. Configure app start address in bootloader_config.h file.

2. APP Project
    a. configure app linker script to the new flash start address.
    b. Provide the IVT offset.
        i. Go to system_stm32f1xx.c file.
        ii. then modify VECT_TAB_OFFSET to be the start address of the flash in APP project.