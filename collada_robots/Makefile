all: installed

SVN_DIR = data
# Should really specify a revision
SVN_REVISION = -r 2700
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave/data
include $(shell rospack find mk)/svn_checkout.mk

INSTALL_DIR=`rospack find collada_robots`

.PHONY: openrave

installed: $(SVN_DIR) patched
	cd $(INSTALL_DIR) && mkdir -p build && cd build && cmake .. && make
	touch installed

clean:
	-make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin

leave_only_products:
	rm -rf $(SVN_DIR)
