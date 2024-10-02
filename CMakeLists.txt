cmake_minimum_required(VERSION 3.10)

project(nthaka)
set(CMAKE_C_STANDARD 99)

include_directories(include include/nthaka/pokecon)
file(GLOB SOURCES "src/*.c")
add_library(nthaka STATIC ${SOURCES})

file(GLOB DOL_SOURCES "src/dol/*.c")
add_library(nthaka_dol STATIC ${DOL_SOURCES})

file(GLOB NXMC2_SOURCES "src/nxmc2/*.c")
add_library(nthaka_nxmc2 STATIC ${NXMC2_SOURCES})

file(GLOB ORCA_SOURCES "src/orca/*.c")
add_library(nthaka_orca STATIC ${ORCA_SOURCES})

file(GLOB POKECON_SOURCES "src/pokecon/*.c")
add_library(nthaka_pokecon STATIC ${POKECON_SOURCES})

# Create test executable
file(GLOB TEST_SOURCES "test/*.c")
add_executable(test ${TEST_SOURCES})
target_link_libraries(test nthaka nthaka_dol nthaka_nxmc2 nthaka_orca nthaka_pokecon)