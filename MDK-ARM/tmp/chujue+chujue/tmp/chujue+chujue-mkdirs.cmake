# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue")
  file(MAKE_DIRECTORY "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue")
endif()
file(MAKE_DIRECTORY
  "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/1"
  "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue"
  "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue/tmp"
  "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue/src/chujue+chujue-stamp"
  "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue/src"
  "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue/src/chujue+chujue-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue/src/chujue+chujue-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/ldpine/Desktop/New Folder/chujue/MDK-ARM/tmp/chujue+chujue/src/chujue+chujue-stamp${cfgdir}") # cfgdir has leading slash
endif()
