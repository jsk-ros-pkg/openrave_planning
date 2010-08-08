all: installed

FILENAME = SoQt-1.4.1.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/SoQt-1.4.1
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH=soqt_qt4.patch
include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && autoconf && ./configure --prefix=`rospack find soqt`
	cd $(SOURCE_DIR) && make && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-make -C $(SOURCE_DIR) clean
	rm -rf $(SOURCE_DIR)
	rm -rf installed

wipe: clean
	rm -rf build patched

wiped: Makefile
	make wipe
	touch wiped

.PHONY : clean download
