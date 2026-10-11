CC=gcc
SRCS = main.c rxtcp.c uart.c
OBJS = $(SRCS:.c=.o)
DEPS = $(SRCS:.c=.d)  # Tracks header dependencies

#CFLAGS=-g -std=c99 -DUART -Wall -Werror
CFLAGS=-g 

all:	truepos_pipe truepos_tcp truepos_uart

%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Include the generated dependency files (ignores them if they don't exist yet)
-include $(DEPS)


truepos_pipe: $(OBJS) truepos.c
	$(CC) $(CFLAGS) -DPIPE $^ -o $@

truepos_tcp: $(OBJS) truepos.c
	$(CC) $(CFLAGS) -DRXTCP $^ -o $@

truepos_uart: $(OBJS) truepos.c
	$(CC) $(CFLAGS) -DUART $^ -o $@

clean:
	rm -f $(OBJS) $(DEPS) truepos_pipe truepos_tcp trupos_uart
