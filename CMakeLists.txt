cmake_minimum_required(VERSION 2.8)
project(reconstruction)

find_package(ecto REQUIRED)

set_ecto_install_package_name(reconstruction)

add_subdirectory(src)

install(DIRECTORY apps/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        PATTERN "*.bag" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
)

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

