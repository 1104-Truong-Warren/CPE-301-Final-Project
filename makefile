final: main.o
	gcc main.o -o final

main.o: main.c
	gcc -c main.c

clean:
	rm *.o final