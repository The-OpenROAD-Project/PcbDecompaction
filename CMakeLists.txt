cmake_minimum_required(VERSION 3.1)

project(PcbDRC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(PCBDRC_HOME ${PROJECT_SOURCE_DIR} )
set(KICADPCB_HOME
  ${PROJECT_SOURCE_DIR}/module/kicadPcb
)
set(GUROBI_HOME
   ~/gurobi901/linux64
)
set(CLP_HOME
   /home1/yenyi/coin-Clp
)



add_subdirectory(${KICADPCB_HOME})

find_package(Boost REQUIRED)

set (PCBDRC_SRC 
    src/decompaction.cpp
    src/gurobiSolver.cpp
    src/clpSolver.cpp
    #src/GridBasedRouter.cpp
    #src/plotter.cpp
    #src/globalParam.cpp
  )

set (PCBDRC_HEADER
    src/decompaction.h
    src/gurobiSolver.h
    src/clpSolver.h
    #src/object.h
    #src/GridBasedRouter.h
    #src/plotter.h
    #src/globalParam.h
    #src/util.h
  )

############################################################
# include directories 
############################################################
include_directories(
    ${Boost_INCLUDE_DIR}
    ${KICADPCB_HOME}/src
    ${GUROBI_HOME}/include
    ${CLP_HOME}/include/coin
  )

############################################################
# Create a library 
############################################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PCBDRC_HOME}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PCBDRC_HOME}/lib)

add_library( pcbdrclib STATIC
    ${PCBDRC_SRC}
)

set_target_properties( pcbdrclib 
  PROPERTIES OUTPUT_NAME "pcbdrc"
)

############################################################
# Create an executable
############################################################
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PCBDRC_HOME}/bin)

add_executable( pcbdrc src/main.cpp )

add_dependencies( pcbdrc kicadpcbparserlib )

target_link_libraries( pcbdrc
  pcbdrclib
  kicadpcbparserlib
  /home1/yenyi/gurobi901/linux64/lib/libgurobi_c++.a
  # ${GUROBI_HOME}/lib/libgurobi_c++.a
  ${GUROBI_HOME}/lib/libgurobi90.so
  ${CLP_HOME}/lib/libClp.so
)

############################################################
# Install  
############################################################

install(FILES ${PCBDRC_HEADER} DESTINATION include)
install(TARGETS pcbdrc DESTINATION bin)
install(TARGETS pcbdrclib DESTINATION lib)

