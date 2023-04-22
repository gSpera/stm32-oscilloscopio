ARM:=arm-none-eabi
CFLAGS:=-O0 -g -mthumb -mtune=cortex-m4 -I Libreria -fno-builtin
LDFLAGS:= /usr/lib/gcc/arm-none-eabi/12.2.0/thumb/v6-m/nofp/libgcc.a
OBJS:=main.o usart.o tft.o delay.o vector_table.o Libreria/system_stm32f30x.o gfx.o
OPENOCD_CFG=/usr/share/openocd/scripts/board/stm32f3discovery.cfg

all: out.bin out.lis

out.bin: out.elf
	$(ARM)-objcopy -O binary $< $@

out.lis: out.elf
	$(ARM)-objdump -S $< > $@

out.elf: $(OBJS)
	$(ARM)-ld -o $@ -T linker.ld $(OBJS) $(LDFLAGS) 

%.o: %.s
	$(ARM)-as -o $@ -c $< 
%.o: %.c %.h
	$(ARM)-gcc -o $@ $(CFLAGS) -c $< 


.PHONY: flash
flash: out.bin
	st-flash write out.bin 0x8000000

.PHONY: debug
debug: out.lis flash
	st-util&
	$(ARM)-gdb out.elf -ex "target extended-remote :4242"
	killall st-util

debug-openocd: out.lis flash
	openocd -f $(OPENOCD_CFG)&
	$(ARM)-gdb out.elf -ex "target extended-remote :3333"

.PHONY: clean
clean:
	rm -rf $(OBJS) out.elf out.bin out.lis

