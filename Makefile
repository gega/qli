#

qli:	qli.h qli.c
	gcc -O2 -Wall -o qli qli.c

test:	qli
	./testprep.sh
