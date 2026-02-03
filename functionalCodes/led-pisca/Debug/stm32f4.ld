/* Linker Script para STM32F4 */
ENTRY(_start)  /* Define a entrada principal para o programa */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1M
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  .text :
  {
    . = ALIGN(4);
    *(.text)    /* Código do programa */
    *(.text*)   
  } > FLASH

  .data : 
  {
    . = ALIGN(4);
    *(.data)
  } > SRAM

  .bss :
  {
    . = ALIGN(4);
    *(.bss)
  } > SRAM

  .stack : 
  {
    . = ALIGN(8);
    PROVIDE(_stack_start = .);
    . += 0x1000;   /* Definindo o tamanho da pilha */
    PROVIDE(_stack_end = .);
  } > SRAM
}

