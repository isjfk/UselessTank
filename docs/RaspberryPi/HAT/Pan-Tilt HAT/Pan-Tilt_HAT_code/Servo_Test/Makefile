#DIR_FONTS = ./Fonts
DIR_SRC = ./Servo
DIR_BIN = ./bin

OBJ_C = $(wildcard ${DIR_SRC}/*.c)
OBJ_O = $(patsubst %.c,${DIR_BIN}/%.o,$(notdir ${OBJ_C}))

TARGET = main
#BIN_TARGET = ${DIR_BIN}/${TARGET}

CC = gcc

DEBUG = -g -O0 -Wall
CFLAGS += $(DEBUG) 

LIB = -lwiringPi

${TARGET}:${OBJ_O}
	$(CC) $(CFLAGS) $(OBJ_O) -o $@ $(LIB)

${DIR_BIN}/%.o : $(DIR_SRC)/%.c
	$(CC) $(CFLAGS) -c $< -o $@ $(LIB)

clean :
	rm $(DIR_BIN)/*.*
	rm $(TARGET)
