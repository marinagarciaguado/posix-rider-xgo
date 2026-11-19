CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -O2

.PHONY: all run run-sim test clean

all: build/inicio_posix build/rider_sim build/rider_bridge_sim

build:
	mkdir -p $@

build/inicio_posix: examples/inicio_posix.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -pthread $(LDFLAGS) -o $@ $< $(LDLIBS)

run: build/inicio_posix
	./build/inicio_posix

build/rider_sim: src/rider_sim.c src/rider_ipc.c include/rider_ipc.h include/rider_protocol.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -Iinclude -pthread $(LDFLAGS) -o $@ src/rider_sim.c src/rider_ipc.c $(LDLIBS) -lrt

build/rider_bridge_sim: src/rider_bridge_sim.c src/rider_ipc.c include/rider_ipc.h include/rider_protocol.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -Iinclude $(LDFLAGS) -o $@ src/rider_bridge_sim.c src/rider_ipc.c $(LDLIBS) -lrt

run-sim: build/rider_sim build/rider_bridge_sim
	./build/rider_sim

build/test_ipc: tests/test_ipc.c src/rider_ipc.c include/rider_ipc.h include/rider_protocol.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -Iinclude $(LDFLAGS) -o $@ tests/test_ipc.c src/rider_ipc.c $(LDLIBS) -lrt

test: build/rider_sim build/rider_bridge_sim build/test_ipc
	./build/test_ipc
	python3 -m unittest discover -s tests -p 'test_*.py' -v

clean:
	$(RM) build/inicio_posix build/rider_sim build/rider_bridge_sim build/test_ipc
