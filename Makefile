CC = gcc
CPPFLAGS += -D_POSIX_C_SOURCE=200809L -Iinclude -Iconfig
CFLAGS += -std=c11 -O2 -Wall -Wextra -Wpedantic -pthread
LDLIBS += -pthread -lrt

SRC = $(wildcard src/*.c)
OBJ = $(SRC:src/%.c=bin/%.o)
TARGET = bin/xgo_supervisor
QUEUE_TEST = bin/test_cmd_queue

.PHONY: all clean test
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

bin/%.o: src/%.c | bin
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

bin:
	mkdir -p $@

$(QUEUE_TEST): tests/test_cmd_queue.c bin/cmd_queue.o bin/scheduler_utils.o bin/ipc_protocol.o
	$(CC) $(CPPFLAGS) $(CFLAGS) -UNDEBUG $(LDFLAGS) $^ $(LDLIBS) -o $@

test: all $(QUEUE_TEST)
	./$(QUEUE_TEST)
	python3 -B -m unittest discover

-include $(OBJ:.o=.d)

clean:
	rm -f $(TARGET) $(QUEUE_TEST) $(OBJ) $(OBJ:.o=.d)
