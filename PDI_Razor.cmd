/* *******************************************************/
/*  C6748.cmd                                            */
/*                                                       */
/* *******************************************************/

--args=64
MEMORY
{
	  FLASH_CS2     org=0x60000000 len=0x02000000 /* AEMIF CS2 region */
	  FLASH_BOOT   	org=0x62000000 len=0x02000000 /* AEMIF CS3 region */
	  FLASH_CS4     org=0x64000000 len=0x02000000 /* AEMIF CS4 region */
}

SECTIONS
{
	.aemif_mem :
	{
	} > FLASH_BOOT, RUN_START(NANDStart)

	"CFG" > DDR_CFG

	"DDR" > DDR

	.ddrram	 :
	{
		. += 0x08000000;
	} > DDR, type=DSECT, RUN_START(EXTERNAL_RAM_START), RUN_END(EXTERNAL_RAM_END)
}

