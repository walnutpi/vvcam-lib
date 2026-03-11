CC = gcc
CFLAGS = -fPIC -shared -g -O0
SRCDIR = src
SOURCEDIR = $(SRCDIR)
INCDIR = include
TARGET = libvvcam.so
SOURCES = $(wildcard $(SOURCEDIR)/*.c)
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET)
install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/lib/
	install -m 644 $(INCDIR)/*.h /usr/local/include/
	mkdir -p /etc/vvcam
	cp -r configs/* /etc/vvcam/

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -I$(INCDIR) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all clean install

$(OBJECTS): | $(INCDIR)
