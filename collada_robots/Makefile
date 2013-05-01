all: installed

GIT_DIR = data/robots
GIT_URL = git://github.com/rdiankov/collada_robots.git
include $(shell rospack find mk)/git_checkout.mk

INSTALL_DIR=`rospack find collada_robots`

.PHONY: openrave

installed: $(GIT_DIR) patched
	cd $(INSTALL_DIR) && mkdir -p build && cd build && cmake .. && make
	touch installed

clean:
	-make -C $(GIT_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(GIT_DIR) include lib share bin

leave_only_products:
	rm -rf $(GIT_DIR)
