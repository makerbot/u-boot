include $(TOPDIR)/config.mk

all: $(obj)uflash

HOSTCFLAGS_NOPED += -I $(SRCTREE)/include -DUSE_HOSTCC

$(obj)uflash: $(SRCTREE)/tools/uflash/uflash.c $(SRCTREE)/lib/crc32.c $(SRCTREE)/lib/errno.c
	$(HOSTCC) $(HOSTCFLAGS_NOPED) $(HOSTLDFLAGS) -o $@ $^

clean:
	rm -f $(obj)uflash

.PHONY: all clean
