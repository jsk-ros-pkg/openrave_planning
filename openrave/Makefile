all: installed

INSTALL_DIR=`rospack find openrave`
SVN_DIR = openrave_svn
# Should really specify a revision
SVN_REVISION = -r 2016
SVN_URL = https://openrave.svn.sourceforge.net/svnroot/openrave/trunk
#SVN_PATCH = fini_patch.patch
include $(shell rospack find mk)/svn_checkout.mk

.PHONY: openrave

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --root)/lib

BULLET_DIR=$(shell rospack find bullet)
BULLET_INCLUDE_DIR=$(shell rospack find bullet)/build/include
BULLET_LIB_DIR=$(shell rospack find bullet)/lib
BULLET_CFLAGS=$(shell rospack export --lang=cpp --attrib=cflags bullet)
BULLET_LFLAGS=$(shell rospack export --lang=cpp --attrib=lflags bullet)

ODE_CFLAGS=$(shell rospack export --lang=cpp --attrib=cflags opende)
ODE_LFLAGS=$(shell rospack export --lang=cpp --attrib=lflags opende)

BUILDDIR=$(shell if [ $(DEBUG) ]; then echo builddebug; else echo build; fi)
CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)

COLLADA_DIR=$(shell rospack find colladadom)

installed: $(SVN_DIR) patched
	echo "prefix=${BULLET_DIR}\n\
libdir=${BULLET_INCLUDE_DIR}\n\
includedir=${BULLET_LIB_DIR}\n\
\n\
Name: Bullet\n\
Description: Bullet as built by ROS\n\
Version: 2.74\n\
Libs: ${BULLET_LFLAGS}\n\
CFlags: ${BULLET_CFLAGS}" > bullet.pc
	cd $(INSTALL_DIR) && cd $(SVN_DIR) && rm -f build/CMakeCache.txt && export PATH="$(shell rospack find soqt)/bin:$(PATH)" && export PKG_CONFIG_PATH="$(PKG_CONFIG_PATH):$(INSTALL_DIR)" && mkdir -p $(BUILDDIR) && cd $(BUILDDIR) && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) -DODE_HAVE_ALLOCATE_DATA_THREAD=1 -D_odeconfig_cflags:STRING="$(ODE_CFLAGS)" -D_odeconfig_lflags:STRING="$(ODE_LFLAGS)" .. && export PARALLEL_JOBS=ROS_PARALLEL_JOBS && make $(ROS_PARALLEL_JOBS) install
	echo "#!/bin/sh\n\
export OPENRAVE_DATA=\$$OPENRAVE_DATA:\`rospack find collada_robots\`/data\n\
export OPENRAVE_PLUGINS=\$$OPENRAVE_PLUGINS:\`rospack find orrosplanning\`/lib\n\
openrave.py \$$* " > $(INSTALL_DIR)/bin/openraveros.py
	echo "#!/bin/sh\n\
export OPENRAVE_DATA=\$$OPENRAVE_DATA:\`rospack find collada_robots\`/data\n\
export OPENRAVE_PLUGINS=\$$OPENRAVE_PLUGINS:\`rospack find orrosplanning\`/lib\n\
openrave \$$* " > $(INSTALL_DIR)/bin/openraveros
	chmod +x $(INSTALL_DIR)/bin/openraveros.py $(INSTALL_DIR)/bin/openraveros
	touch installed

clean:
	-make -C $(SVN_DIR) clean
	rm -f installed

wipe: clean
	rm -rf $(SVN_DIR) include lib share bin

leave_only_products:
	rm -rf $(SVN_DIR)
