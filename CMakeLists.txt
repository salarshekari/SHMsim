cmake_minimum_required(VERSION 3.0)

project(QMars)

set(Sources lib/EOM.cpp
           lib/FGColumnVector3.cpp)

set(Headers inc/EOM.h
            inc/FGColumnVector3.h)
                

add_library(EOMlib STATIC ${Headers} ${Sources})

target_include_directories(EOMlib PUBLIC ${PROJECT_SOURCE_DIR}/inc)

add_executable(QMars main.cpp)

target_link_libraries(QMars PRIVATE EOMlib)