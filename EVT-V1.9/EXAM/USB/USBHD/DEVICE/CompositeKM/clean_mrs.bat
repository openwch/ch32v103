del *.mk /s
del *.elf /s
del *.lst /s
del *.map /s
del *.d /s
del *.o /s

rd .\obj\Core /s/q
rd .\obj\Debug /s/q
rd .\obj\Peripheral /s/q
rd .\obj\Startup /s/q
rd .\obj\Udisk_Lib /s/q
rd .\obj\User /s/q

rd .\obj /s/q

exit