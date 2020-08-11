# Converting of NVM (N-View match) to OpenMVG binary format

project(nvm_to_openmvg)

set(NVM_TO_OPENMVG_SOURCES
    nvm_to_openmv/nvm_to_openmvg.cpp
    nvm_to_openmv/ioutil.cpp)
include_directories(nvm_to_openmv/io)

set(EXECUTABLE_NAME ${PROJECT_NAME})

find_package(Eigen3 QUIET)

#EIGEN COMES THROUGH OpenMVG CMake transitivity!
#if (EIGEN_INCLUDE_DIR_HINTS)
#        include_directories(${EIGEN_INCLUDE_DIR_HINTS})
#else()
#        if (NOT EIGEN3_FOUND)
#                message(SEND_ERROR "EIGEN3 NOT FOUND")
#        else()
#                include_directories(${EIGEN3_INCLUDE_DIR})
#        endif()
#endif()

find_package(OpenMVG REQUIRED)
find_package(OSG 3.5.6 REQUIRED)
find_package(OpenThreads REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.60 COMPONENTS filesystem system regex REQUIRED)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()


include_directories(${OPENMVG_INCLUDE_DIRS})
MESSAGE("OpenMVG includes: " ${OPENMVG_INCLUDE_DIRS})
include_directories(${OPENMVG_INCLUDE_DIRS}/third_party)
include_directories(${OSG_INCLUDE_DIRS})

add_executable(${EXECUTABLE_NAME} ${NVM_TO_OPENMVG_SOURCES} ${INCLUDES})

target_link_libraries(${EXECUTABLE_NAME}
    OpenMVG::openMVG_easyexif
    OpenMVG::openMVG_exif
    OpenMVG::openMVG_fast
    OpenMVG::openMVG_features
    OpenMVG::openMVG_geometry
    OpenMVG::openMVG_image
    OpenMVG::openMVG_kvld
    OpenMVG::openMVG_lemon
    OpenMVG::openMVG_linearProgramming
    OpenMVG::openMVG_lInftyComputerVision
    OpenMVG::openMVG_matching
    OpenMVG::openMVG_matching_image_collection
    OpenMVG::openMVG_multiview
    OpenMVG::openMVG_numeric
    OpenMVG::openMVG_robust_estimation
    OpenMVG::openMVG_sfm
    OpenMVG::openMVG_stlplus
    OpenMVG::openMVG_system)
if(Boost_FOUND)
    target_link_libraries(${EXECUTABLE_NAME} ${Boost_LIBRARIES})
endif()
target_link_libraries(${EXECUTABLE_NAME} ${OSG_LIBRARY})
target_link_libraries(${EXECUTABLE_NAME} ${OSGDB_LIBRARY})
target_link_libraries(${EXECUTABLE_NAME} ${OPENTHREADS_LIBRARIES})

install(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION bin)
