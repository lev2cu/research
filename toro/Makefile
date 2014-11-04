all: installed

SVN_DIR = build/toro-svn
BUILD_DIR = $(SVN_DIR)/trunk
SVN_URL = https://svn.openslam.org/data/svn/toro
SVN_REVISION = -r 28
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR)
	mkdir -p $(SVN_DIR)
	mkdir -p ./include/ccny_toro
	mkdir -p ./lib
	cd $(BUILD_DIR) && make
	cp $(BUILD_DIR)/*.hh ./include/ccny_toro
	cp $(BUILD_DIR)/*.hxx ./include/ccny_toro
	cp $(BUILD_DIR)/*.a ./lib
	touch installed

clean:
	cd $(BUILD_DIR) && make clean
	rm -rf installed patched

wipe: clean
	rm -rf ./include ./lib $(SVN_DIR)  

