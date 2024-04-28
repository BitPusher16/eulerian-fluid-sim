
# gcc -o bin/sim src/sim.c
# gcc -o bin/sim src/sim.c -I include

# gcc -o bin/sim src/sim.c src/misc.c -I include -I /usr/local/include -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
sim: src/sim.c
	gcc -o bin/sim src/sim.c src/misc.c \
		-I include -I /usr/local/include \
		-lraylib -lGL -lm -lpthread -ldl -lrt -lX11
