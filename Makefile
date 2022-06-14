.PHONY: build

build: ncsu_CoEst.c
	g++ ncsu_CoEst.c -l modbus -l sqlite3 -l curl -L /usr/local -o bmserver

default: build
