add_executable( bba simple_bundle_adjuster.cpp )

# external libs
find_package(Ceres REQUIRED)
# Or find_package(Ceres REQUIRED PATHS "/path/to/install/dir") if the install directory is not in your PATH

target_include_directories(bba
  PRIVATE
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(bba
  PRIVATE
    ${CERES_LIBRARIES}
)