# # Look for libnl libraries
# PKG_CONFIG ?= pkg-config
# NL2FOUND := $(shell $(PKG_CONFIG) --atleast-version=2 libnl-2.0 && echo Y)
# NL3FOUND := $(shell $(PKG_CONFIG) --atleast-version=3 libnl-3.0 && echo Y)
# NL31FOUND := $(shell $(PKG_CONFIG) --exact-version=3.1 libnl-3.1 && echo Y)
# NL3xFOUND := $(shell $(PKG_CONFIG) --atleast-version=3.2 libnl-3.0 && echo Y)

# CFLAGS = -g -std=gnu11 -Wall -Wextra -Wno-unused-parameter -O2
# LDFLAGS = -levent -lm

# ifeq ($(NL2FOUND),Y)
# CFLAGS += -DCONFIG_LIBNL20
# LDFLAGS += -lnl-genl
# NLLIBNAME = libnl-2.0
# endif

# ifeq ($(NL3xFOUND),Y)
# # libnl 3.2 might be found as 3.2 and 3.0
# NL3FOUND = N
# CFLAGS += -DCONFIG_LIBNL30
# LDFLAGS += -lnl-genl-3
# NLLIBNAME = libnl-3.0
# endif

# ifeq ($(NL3FOUND),Y)
# CFLAGS += -DCONFIG_LIBNL30
# LDFLAGS += -lnl-genl
# NLLIBNAME = libnl-3.0
# endif

# # nl-3.1 has a broken libnl-gnl-3.1.pc file
# # as show by pkg-config --debug --libs --cflags --exact-version=3.1 libnl-genl-3.1;echo $?
# ifeq ($(NL31FOUND),Y)
# CFLAGS += -DCONFIG_LIBNL30
# LDFLAGS += -lnl-genl
# NLLIBNAME = libnl-3.1
# endif

# ifeq ($(NLLIBNAME),)
# $(error Cannot find development files for any supported version of libnl)
# endif

# LDFLAGS += $(shell $(PKG_CONFIG) --libs $(NLLIBNAME))
# CFLAGS += $(shell $(PKG_CONFIG) --cflags $(NLLIBNAME))

# LDFLAGS+=-lconfig -lpthread
# OBJECTS=yawmd.o yserver.o config.o per.o config_dynamic.o yserver_messages.o yserver_messages_network.o



PKG_CONFIG ?= pkg-config

CFLAGS = -g0 -std=gnu17 -Wall -Wextra -Wno-unused-parameter -O3
LDFLAGS = -levent -lm -lconfig -lpthread

NLLIBNAME = libnl-3.0 libnl-genl-3.0
LDFLAGS += $(shell $(PKG_CONFIG) --libs $(NLLIBNAME))
CFLAGS += $(shell $(PKG_CONFIG) --cflags $(NLLIBNAME))

OBJECTS=yawmd.o config.o per.o

all: yawmd 

yawmd: $(OBJECTS) 
	$(CC) -o $@ $(OBJECTS) $(LDFLAGS) 
 
clean: 
	rm -f $(OBJECTS) yawmd
