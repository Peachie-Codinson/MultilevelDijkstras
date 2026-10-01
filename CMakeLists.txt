cmake_minimum_required (VERSION 4.0)

project (Multi_objective_search)

set(CMAKE_CXX_STANDARD 20)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare -Wno-reorder") #new
#--fno-omit-frame-pointer --mno-omit-leaf-frame-pointer")                                                                                                                                                                                                                                                                                                                                               `v

include_directories("inc")

file(GLOB SOURCES "src/BiCriteria/*.cpp" "src/Utils/*.cpp")

# Find Boost
#set(BOOST_ROOT "/opt/homebrew/Cellar/boost/1.89.0/")
#set(BOOST_INCLUDE_DIR "/opt/homebrew/Cellar/boost/1.89.0/include/boost/")
#set(BOOST_LIBRARY_DIR "/opt/homebrew/Cellar/boost/1.89.0/lib/")
#list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/boost/1.89.0/")
find_package( Boost REQUIRED COMPONENTS program_options log thread filesystem)

message(${CMAKE_BINARY_DIR})

add_executable(multiobj src/driver.cpp ${SOURCES}
        src/driver.cpp
)

target_link_libraries(multiobj ${Boost_LIBRARIES})
target_include_directories(multiobj PRIVATE ${Boost_INCLUDE_DIRS} )