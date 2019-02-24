clean:
	rm -rf output
	rm -f log.txt
	mkdir output

cc:clean
	gcc -c ./src/emulator_main.c -o ./output/emulator_main.o -I ./include
	gcc -c ./src/6502.c -o ./output/6502.o -I ./include
	gcc -c ./src/file.c -o ./output/file.o -I ./include
	gcc ./output/emulator_main.o ./output/6502.o ./output/file.o -o ./output/emu.o -I ./include

rr:
	./output/emu.o
