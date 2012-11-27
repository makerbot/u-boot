MEMORY { .sram : ORIGIN = 0x80000000, LENGTH = 32768 }
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
 . = 0x00000000;
 . = ALIGN(4);
 .text :
 {
 __start = .;
   arch/arm/cpu/arm926ejs/start.o (.text)
   *(.text*)
 } >.sram
 . = ALIGN(4);
 .rodata : { *(SORT_BY_ALIGNMENT(.rodata*)) } >.sram
 . = ALIGN(4);
 .data : { *(SORT_BY_ALIGNMENT(.data*)) } >.sram
 . = ALIGN(4);
 .rel.dyn : {
  __rel_dyn_start = .;
  *(.rel*)
  __rel_dyn_end = .;
 } >.sram
 .dynsym : {
  __dynsym_start = .;
  *(.dynsym)
 } >.sram
 .bss :
 {
  . = ALIGN(4);
  __bss_start = .;
  *(.bss*)
  . = ALIGN(4);
  __bss_end__ = .;
 } >.sram
 __image_copy_end = .;
 _end = .;
}
