/*
 * Do not modify this file; it is automatically generated from the template
 * linkcmd.xdt in the ti.targets.elf package and will be overwritten.
 */

/*
 * put '"'s around paths because, without this, the linker
 * considers '-' as minus operator, not a file name character.
 */


-l"C:\hzhou39_decamar2_ashwina2\SE423Repo\Contest\FinalContestRobot2\FinalContestRobot2Project\Debug\configPkg\package\cfg\FinalContestRobot2_pe674.oe674"
-l"C:\hzhou39_decamar2_ashwina2\SE423Repo\Contest\FinalContestRobot2\SYSBIOS\src\sysbios\sysbios.ae674"
-l"C:\CCStudio_v8\bios_6_70_01_03\packages\ti\targets\rts6000\lib\ti.targets.rts6000.ae674"
-l"C:\CCStudio_v8\bios_6_70_01_03\packages\ti\targets\rts6000\lib\boot.ae674"

--retain="*(xdc.meta)"


--args 0x0
-heap  0x0
-stack 0x2000

MEMORY
{
    IROM (RX) : org = 0x11700000, len = 0x100000
    IRAM (RWX) : org = 0x11800000, len = 0x38000
    L3_CBA_RAM (RWX) : org = 0x80000000, len = 0x20000
    DDR (RWX) : org = 0xc3000000, len = 0x1000000
}

/*
 * Linker command file contributions from all loaded packages:
 */

/* Content from xdc.services.global (null): */

/* Content from xdc (null): */

/* Content from xdc.corevers (null): */

/* Content from xdc.shelf (null): */

/* Content from xdc.services.spec (null): */

/* Content from xdc.services.intern.xsr (null): */

/* Content from xdc.services.intern.gen (null): */

/* Content from xdc.services.intern.cmd (null): */

/* Content from xdc.bld (null): */

/* Content from ti.targets (null): */

/* Content from ti.targets.elf (null): */

/* Content from xdc.rov (null): */

/* Content from xdc.runtime (null): */

/* Content from ti.targets.rts6000 (null): */

/* Content from ti.sysbios.interfaces (null): */

/* Content from ti.sysbios.family (null): */

/* Content from xdc.services.getset (null): */

/* Content from ti.sysbios.family.c62 (null): */

/* Content from xdc.runtime.knl (null): */

/* Content from ti.sysbios.family.c64p.primus (null): */

/* Content from ti.catalog.c6000 (null): */

/* Content from ti.catalog (null): */

/* Content from ti.catalog.peripherals.hdvicp2 (null): */

/* Content from xdc.platform (null): */

/* Content from xdc.cfg (null): */

/* Content from ti.platforms.generic (null): */

/* Content from ti.platforms.evmOMAPL138 (null): */

/* Content from ti.sysbios.hal (null): */

/* Content from ti.sysbios (null): */

/* Content from ti.sysbios.rts (ti/sysbios/rts/linkcmd.xdt): */

/* Content from ti.sysbios.rts.ti (ti/sysbios/rts/ti/linkcmd.xdt): */

/* Content from ti.sysbios.knl (null): */

/* Content from ti.sysbios.timers.timer64 (null): */

/* Content from ti.sysbios.family.c64p (ti/sysbios/family/c64p/linkcmd.xdt): */
ti_sysbios_family_c64p_Cache_l1dSize = 32768;
ti_sysbios_family_c64p_Cache_l1pSize = 32768;
ti_sysbios_family_c64p_Cache_l2Size = 32768;

/* Content from ti.sysbios.gates (null): */

/* Content from ti.sysbios.xdcruntime (null): */

/* Content from ti.sysbios.heaps (null): */

/* Content from ti.sysbios.utils (null): */

/* Content from configPkg (null): */

/* Content from xdc.services.io (null): */


/*
 * symbolic aliases for static instance objects
 */
xdc_runtime_Startup__EXECFXN__C = 1;
xdc_runtime_Startup__RESETFXN__C = 1;

SECTIONS
{
    .text: load >> IRAM
    .ti.decompress: load > IRAM
    .stack: load > IRAM
    GROUP: load > IRAM
    {
        .bss:
        .neardata:
        .rodata:
    }
    .cinit: load > IRAM
    .pinit: load >> IRAM
    .init_array: load > IRAM
    .const: load >> IRAM
    .data: load >> IRAM
    .fardata: load >> IRAM
    .switch: load >> IRAM
    .sysmem: load > IRAM
    .far: load >> IRAM
    .args: load > IRAM align = 0x4, fill = 0 {_argsize = 0x0; }
    .cio: load >> IRAM
    .ti.handler_table: load > IRAM
    .c6xabi.exidx: load > IRAM
    .c6xabi.extab: load >> IRAM
    .vecs: load > IRAM
    xdc.meta: load > IRAM, type = COPY

}
