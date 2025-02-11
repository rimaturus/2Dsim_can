TARGET 	= 2D_sim
CC 		= gcc
CFLAGS 	= -g -O2
LIBS 	= `allegro-config --libs` -lyaml -lm -lpthread

SRCS 	= $(wildcard src/*.c)
OBJS 	= $(SRCS:.c=.o)

INC_DIR = include
CFLAGS += -I$(INC_DIR)

# Default
all: $(TARGET)

# Compile
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

# Compile step: pattern rule for building .o from .c
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(LIBS)

# Clean
clean:
	rm -f $(OBJS) $(TARGET)