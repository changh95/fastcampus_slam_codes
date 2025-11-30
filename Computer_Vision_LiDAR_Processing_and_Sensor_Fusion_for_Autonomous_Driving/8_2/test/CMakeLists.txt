# Sources
set(TEST_SRCS
    StringReturnerTest.cpp
)
    
# include
include_directories(
        ${PROJECT_SOURCE_DIR}/src
)

# Targets
add_executable(
        unitTests
        GTestMain.cpp # Main entry for unit tests
        ${TEST_SRCS}
)

target_link_libraries(
        unitTests
        ${GTEST_LIBS}
)

set_target_properties(unitTests
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "../"
        )

add_test(NAME unit COMMAND ${CMAKE_BINARY_DIR}/unitTests)
