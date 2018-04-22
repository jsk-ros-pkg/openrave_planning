^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openrave
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#20 <https://github.com/jsk-ros-pkg/openrave_planning/issues/20>`_ from k-okada/master
  update  openrave package as of 2017/02/19
* remove collada_robots from build_depend
* update openrave to Feb/19/2018 versoin, which does not require sympy==0.7.1, as described in https://github.com/rdiankov/openrave/issues/375#issuecomment-155994724
* Merge pull request `#18 <https://github.com/jsk-ros-pkg/openrave_planning/issues/18>`_ from k-okada/fix_catkin_source
  do not import openravepy if it working... need to this when deb settings
* do not import openravepy if it working... need to this when deb settings
* Merge pull request `#17 <https://github.com/jsk-ros-pkg/openrave_planning/issues/17>`_ from k-okada/add_travis_kinetic
  run all test within docker, specially for kinetic
* compile openrave with -Wno-deprecated
* add libxml2
* Contributors: Kei Okada

0.0.5 (2017-02-03)
------------------
* re-enable opende in package.xml
* 99.openarve.sh.in: openrave-config is now global executive
* use production release of Jan/19/2017
* Contributors: Kei Okada

0.0.4 (2017-01-19)
------------------
* use dist-packages as python install dir instad of site-packages
* Contributors: Kei Okada

0.0.3 (2016-05-27)
------------------
* CMakeLists.txt : fix compiile scheme for install/devel
* Contributors: Kei Okada

0.0.2 (2016-05-25)
------------------

0.0.1 (2016-05-25)
------------------
* add 99.openrave.sh
* update openrave/collada_robots package to catkin
* Contributors: Kei Okada
