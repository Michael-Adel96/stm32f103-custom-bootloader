## Usage steps
1. BL Project<br>
    >>a. Configure app start address in bootloader_config.h file.<br>

2. APP Project<br>
    >>a. configure app linker script to the new flash start address.<br>
    >>b. Provide the IVT offset.<br>
    >>>=> Go to system_stm32f1xx.c file.<br>
    >>>=> Uncomment this line "#define USER_VECT_TAB_ADDRESS"<br>
    >>>=> then modify VECT_TAB_OFFSET to be the start address of the flash in APP project.<br>

## Flash memory organization
![Flash memory organization](https://github.com/Michael-Adel96/stm32f103-custom-bootloader/blob/basic_func_BL/Flash%20organization/Flash%20organization.drawio.png)
