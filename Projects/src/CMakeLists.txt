cmake_minimum_required(VERSION 3.5)
# Find eigen in system directory and treat it as an interface library
find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE IMPORTED)# Generate the local Makefile
if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

add_cispba_executable(src main.cpp
        globalincludes.h
        components/Agents.h
        components/Agents.cpp
        components/Scene.h
        components/Scene.cpp
        constraints/Constraint.h
        constraints/FrictionalConstraint.h
        constraints/FrictionalConstraint.cpp
        constraints/CollisionAvoidanceConstraint.cpp
        constraints/Constraint.cpp
        integrator/BaseIntegrator.h
        integrator/BaseIntegrator.cpp
        integrator/ExpIntegrator.h
        integrator/ExpIntegrator.cpp
        Solver.h
        Solver.cpp
        CrowdSim.cpp)

target_include_directories(src SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

# this flag will help to make it faster - @anantha
set(CMAKE_CXX_FLAGS "-O3")

target_link_libraries(src partio)
