all: installed

SVN_DIR = data
# Should really specify a revision
SVN_REVISION = -r 2457
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave/data
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

installed: $(SVN_DIR) patched
	touch installed

clean:
	-make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin

leave_only_products:
	rm -rf $(SVN_DIR)
