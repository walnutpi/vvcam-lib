CC = gcc
CFLAGS = -fPIC -shared -g -O0
SRCDIR = src
SOURCEDIR = $(SRCDIR)
INCDIR = include
TARGET = libvvcam.so
SOURCES = $(wildcard $(SOURCEDIR)/*.c)
OBJECTS = $(SOURCES:.c=.o)

# 新增 dectet 相关变量
DECTET_DIR = dectet
DECTET_SOURCES = $(wildcard $(DECTET_DIR)/*.c)
DECTET_OBJECTS = $(DECTET_SOURCES:.c=.o)
DECTET_TARGET = dectet_sensor

all: $(TARGET) $(DECTET_TARGET)

install: install-lib install-dectet install-configs

install-lib: $(TARGET)
	install -m 755 $(TARGET) /usr/lib/
	install -m 644 $(INCDIR)/*.h /usr/include/

install-dectet: $(DECTET_TARGET)
	install -m 755 $(DECTET_TARGET) /usr/bin/

install-configs:
	mkdir -p /etc/vvcam
	cp -r configs/* /etc/vvcam/

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

$(DECTET_TARGET): $(DECTET_OBJECTS)
	$(CC) -o $@ $^ -I$(INCDIR)

%.o: %.c
	$(CC) $(CFLAGS) -I$(INCDIR) -c $< -o $@

$(DECTET_DIR)/%.o: $(DECTET_DIR)/%.c
	$(CC) $(filter-out -fPIC -shared, $(CFLAGS)) -I$(INCDIR) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(DECTET_OBJECTS) $(TARGET) $(DECTET_TARGET)

.PHONY: all clean install install-lib install-dectet install-configs

$(OBJECTS) $(DECTET_OBJECTS): | $(INCDIR)