/******************************************************************************
 *  Based on Energia's ld script
 ******************************************************************************/

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00100000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
}

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        _text = .;
        KEEP(*(.isr_vector))
        *(.text .text* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(.rodata .rodata* .gnu.linkonce.r.*)
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        
        . = ALIGN(4);
        KEEP(*(.init))

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP(*(SORT(.preinit_array*)))
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
        
        . = ALIGN(4);
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
        
        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end = .;

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        . = ALIGN(4);
        _data = .;
        *(vtable)
        *(.data .data* .gnu.linkonce.d.*)
        _edata = .;
    } > SRAM
    
    .bss (NOLOAD):
    {
        . = ALIGN(4);
        _bss = .;
        __bss_start__ = _bss;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4); 
        _ebss = .;
        __bss_end__ = _ebss;
        . = ALIGN(8);
    } > SRAM
    
    PROVIDE_HIDDEN(__exidx_start = .);
    .ARM.exidx :
    {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > SRAM
    PROVIDE_HIDDEN(__exidx_end = .);
    
    . = ALIGN(4);
    _end = . ;
}

/* end of allocated ram is start of heap, heap grows up towards stack*/
PROVIDE(end = _end);

/* top of stack starts at end of ram, stack grows down towards heap */
PROVIDE(_estack = ORIGIN(SRAM) + LENGTH(SRAM));
