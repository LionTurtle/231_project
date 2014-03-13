include CommonDefs.mak

BIN_DIR = Bin

INC_DIRS = \
	../../Include \
	../../ThirdParty/GL/ \
	../Common \
	./

SRC_FILES = *.cpp

ifeq ("$(OSTYPE)","Darwin")
	CFLAGS += -DMACOS
	LDFLAGS += -framework OpenGL -framework GLUT
else
	CFLAGS += -DUNIX -DGLX_GLXEXT_LEGACY
	USED_LIBS += glut GL GLU
endif

LIB_DIRS += $(OPENNI2_REDIST)
LIB_DIRS += $(NITE2_REDIST)

INC_DIRS += $(OPENNI2_INCLUDE)
INC_DIRS += $(NITE2_INCLUDE)

USED_LIBS += OpenNI2 NiTE2

EXE_NAME = UserViewer

CFLAGS += -Wall

include CommonCppMakefile

.PHONY: copy-redist
copy-redist:
	cp -R $(OPENNI2_REDIST)/* $(OUT_DIR)
	
$(OUTPUT_FILE): copy-redist

.PHONY: copy-redist
copy-NITE2-redist:
	cp -R $(NITE2_REDIST)/* $(OUT_DIR)

$(OUTPUT_FILE): copy-NITE2-redist