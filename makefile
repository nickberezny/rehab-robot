ifeq ($(OS),Windows_NT)
  ifeq ($(shell uname -s),) # not in a bash-like shell
	CLEANUP = del /F /Q
	MKDIR = mkdir
  else # in a bash-like shell, like msys
	CLEANUP = rm -f
	MKDIR = mkdir -p
  endif
	TARGET_EXTENSION=.exe
else
	CLEANUP = rm -f
	MKDIR = mkdir -p
	TARGET_EXTENSION=out
endif

.PHONY: all build clean
.PHONY: clean
.PHONY: test

PATHU = unity/src/
PATHS = src/
PATHT = test/
PATHB = build/
PATHD = build/depends/
PATHO = build/objs/
PATHR = build/results/

PATHUI = src/UI/

BUILD_PATHS = $(PATHB) $(PATHD) $(PATHO) $(PATHR)

SRCT = $(wildcard $(PATHT)*.c)

COMPILE=gcc -c 
LINK=gcc
DEPEND=gcc -MM -MG -MF 
CFLAGS=-I. -I$(PATHU) -I$(PATHS) -DTEST 
LIB=-lm -lpthread -lLabJackM -ltensorflow 

RESULTS = $(patsubst $(PATHT)Test%.c,$(PATHR)Test%.txt,$(SRCT) )

PASSED = `grep -s PASS $(PATHR)*.txt`
FAIL = `grep -s FAIL $(PATHR)*.txt`
IGNORE = `grep -s IGNORE $(PATHR)*.txt`

FILES = $(wildcard $(PATHS)*.c)
FILES := $(filter-out %TestDaq.c, $(FILES))
FILES := $(filter-out %SPI_Test.c, $(FILES))
FILES := $(filter-out %I2C_Test.c, $(FILES))
FILES := $(filter-out %UART_Test.c, $(FILES))

build:
	sudo gcc $(FILES)  -o robotController $(LIB)

docs:
	sudo doxygen doxygen_config
	firefox doc/html/index.html

daq: 
	sudo gcc $(PATHS)TestDaq.c $(PATHS)Daq.c $(PATHS)TimeUtilities.c -o testDaq $(LIB)

spi:
	sudo gcc $(PATHS)SPI_Test.c -o spiTest $(LIB)

i2c: 
	sudo gcc $(PATHS)I2C_Test.c -o i2cTest $(LIB)

uart: 
	sudo gcc $(PATHS)UART_Test.c -o uartTest $(LIB)

test: $(BUILD_PATHS) $(RESULTS)
	@echo "-----------------------\nIGNORES:\n-----------------------"
	@echo "$(IGNORE)"
	@echo "-----------------------\nFAILURES:\n-----------------------"
	@echo "$(FAIL)"
	@echo "-----------------------\nPASSED:\n-----------------------"
	@echo "$(PASSED)"
	@echo "\nDONE"

$(PATHR)%.txt: $(PATHB)%.$(TARGET_EXTENSION)
	-./$< > $@ 2>&1

$(PATHB)Test%.$(TARGET_EXTENSION): $(PATHO)Test%.o $(PATHO)%.o $(PATHU)unity.o #$(PATHD)Test%.d
	$(LINK) -o $@ $^ $(LIB)

$(PATHO)%.o:: $(PATHT)%.c
	$(COMPILE) $(CFLAGS) $< -o $@ $(LIB)

$(PATHO)%.o:: $(PATHS)%.c
	$(COMPILE) $(CFLAGS) $< -o $@ $(LIB)

$(PATHO)%.o:: $(PATHU)%.c $(PATHU)%.h
	$(COMPILE) $(CFLAGS) $< -o $@ $(LIB)

$(PATHD)%.d:: $(PATHT)%.c
	$(DEPEND) $@ $<

$(PATHB):
	$(MKDIR) $(PATHB)

$(PATHD):
	$(MKDIR) $(PATHD)

$(PATHO):
	$(MKDIR) $(PATHO)

$(PATHR):
	$(MKDIR) $(PATHR)

clean:
	$(CLEANUP) $(PATHO)*.o
	$(CLEANUP) $(PATHB)*.$(TARGET_EXTENSION)
	$(CLEANUP) $(PATHR)*.txt

.PRECIOUS: $(PATHB)Test%.$(TARGET_EXTENSION)
.PRECIOUS: $(PATHD)%.d
.PRECIOUS: $(PATHO)%.o
.PRECIOUS: $(PATHR)%.txt