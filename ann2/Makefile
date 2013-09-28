all: installed

INSTALL_DIR=`rospack find ann2`
TARBALL = ann_1.1.1.tar.gz
#TARBALL_URL = 
SOURCE_DIR = build/ann_1.1.1
MD5SUM_FILE = ann_1.1.1.tar.gz.md5sum
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

PATCH=ann.patch

installed: wiped $(SOURCE_DIR)/unpacked src/setup_ann.py src/ann/ann_wrap.cpp
	cd $(INSTALL_DIR) && cd $(SOURCE_DIR) && patch -s -N -p1 < ../../$(PATCH) || echo
	cd $(INSTALL_DIR) && cd $(SOURCE_DIR)/src && make targets "ANNLIB = libANN.a" "C++ = g++" "CFLAGS = -fPIC -O3" "MAKELIB = ar ruv" "RANLIB = true"
	cd $(INSTALL_DIR) && cp -rf $(SOURCE_DIR)/include $(INSTALL_DIR)
	cd $(INSTALL_DIR) && mkdir -p lib && cp -rf $(SOURCE_DIR)/lib/libANN.a $(INSTALL_DIR)/lib/libANN_1_1_1.a
	cd $(INSTALL_DIR)/src && python setup_ann.py build --build-base=$(INSTALL_DIR)/src/build install --home=$(INSTALL_DIR)/src
	cd $(INSTALL_DIR) && touch installed

wiped: Makefile
	cd $(INSTALL_DIR) && make wipe
	cd $(INSTALL_DIR) && touch wiped
	cd $(INSTALL_DIR) && mkdir build

clean:
	-cd $(INSTALL_DIR) && make -C $(SOURCE_DIR) clean
	cd $(INSTALL_DIR) && rm -rf $(SOURCE_DIR) include lib
	cd $(INSTALL_DIR) && rm -rf installed

wipe: clean
	cd $(INSTALL_DIR) && rm -rf build

