
rmdir /s /q ".\debug"
rmdir /s /q ".\release\core"
rmdir /s /q ".\release\drivers"
rmdir /s /q ".\release\USB"

del /s .\release\*.elf
del /s .\release\*.list
del /s .\release\*.map
del /s .\release\*.mk
del /s .\release\makefile

::pause
