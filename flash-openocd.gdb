# 1) start openocd in other terminal
#    $ openocd -f board/stm32f3discovery.cfg
# 2) arm-none-eabi-gdb -x flash-openocd.gdb 

target extended-remote :3333
exec build/ch.elf 
load
continue
quit
