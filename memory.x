
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   : ORIGIN = 0x24000000, LENGTH = 128K
}

_stext = ALIGN(ADDR(.vector_table) + SIZEOF(.vector_table), 8);
