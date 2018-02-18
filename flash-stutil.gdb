# 1) start st-util in background
# 2) arm-none-eabi-gdb -x flash-stutil.gdb 

target extended-remote :4242
exec build/ch.elf 
load
quit
