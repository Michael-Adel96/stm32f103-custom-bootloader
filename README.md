## Usage steps
1. BL Project<br>
    a. Configure app_1 and app_2 start addresses in bootloader_config.h file.<br>

2. APP Project<br>
    a. Configure app linker script to the new flash start address.<br>
    b. Configure the IVT offset.<br>
    >-> Go to system_stm32f1xx.c file.<br>
    >-> Uncomment this line "#define USER_VECT_TAB_ADDRESS"<br>
    >-> then modify VECT_TAB_OFFSET to be the start address of the flash in APP project.<br>

## Flash memory organization
<p align="center">
  <img src="https://github.com/Michael-Adel96/stm32f103-custom-bootloader/blob/basic_func_BL/Flash%20organization/Flash%20organization.drawio.png" alt="Flash memory organization">
</p>
