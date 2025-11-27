# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BUILD_SOURCE_DIRS "/home/isro/isro_ws/src/rtabmap_bk;/home/isro/isro_ws/build/rtabmap")
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_COMPONENTS_ALL "Unspecified;devel;runtime")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "/opt/cmake-3.24.2/share/cmake-3.24/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "RTABMap built using CMake")
set(CPACK_DMG_SLA_USE_RESOURCE_FILE_LICENSE "ON")
set(CPACK_GENERATOR "TBZ2")
set(CPACK_INSTALL_CMAKE_PROJECTS "/home/isro/isro_ws/build/rtabmap;RTABMap;ALL;/")
set(CPACK_INSTALL_PREFIX "/home/isro/isro_ws/devel")
set(CPACK_MODULE_PATH "/home/isro/isro_ws/src/rtabmap_bk/cmake_modules;/usr/lib/cmake/vtk-6.3")
set(CPACK_NSIS_DISPLAY_NAME "RTABMap 0.20.9")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "RTABMap 0.20.9")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_OUTPUT_CONFIG_FILE "/home/isro/isro_ws/build/rtabmap/CPackConfig.cmake")
set(CPACK_PACKAGE_CONTACT "matlabbe@gmail.com")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/opt/cmake-3.24.2/share/cmake-3.24/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RTAB-MAP is a Real-Time Appearance-Based Mapping approach.")
set(CPACK_PACKAGE_FILE_NAME "RTABMap-0.20.9-Linux")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "RTABMap 0.20.9")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "RTABMap 0.20.9")
set(CPACK_PACKAGE_NAME "RTABMap")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "RTABMap project")
set(CPACK_PACKAGE_VERSION "0.20.9")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "20")
set(CPACK_PACKAGE_VERSION_PATCH "9")
set(CPACK_RESOURCE_FILE_LICENSE "/home/isro/isro_ws/src/rtabmap_bk/LICENSE")
set(CPACK_RESOURCE_FILE_README "/opt/cmake-3.24.2/share/cmake-3.24/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/opt/cmake-3.24.2/share/cmake-3.24/Templates/CPack.GenericWelcome.txt")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "ZIP")
set(CPACK_SOURCE_IGNORE_FILES "\\.svn/;\\.settings/;/home/isro/isro_ws/src/rtabmap_bk/build/[a-zA-Z0-9_]+;~$;/home/isro/isro_ws/src/rtabmap_bk/bin/.*rtabmap;/home/isro/isro_ws/src/rtabmap_bk/bin/.*RTABMap;/home/isro/isro_ws/src/rtabmap_bk/bin/.*[tT]est;/home/isro/isro_ws/src/rtabmap_bk/bin/.*[eE]xample;/home/isro/isro_ws/src/rtabmap_bk/bin/.*uresourcegenerator;\\.DS_Store")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/isro/isro_ws/build/rtabmap/CPackSourceConfig.cmake")
set(CPACK_SYSTEM_NAME "Linux")
set(CPACK_THREADS "1")
set(CPACK_TOPLEVEL_TAG "Linux")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/isro/isro_ws/build/rtabmap/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
