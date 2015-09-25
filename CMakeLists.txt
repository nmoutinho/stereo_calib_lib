cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME STEREO_CALIB_LIB)
PROJECT(${PROJECTNAME})

find_package(OpenCV)

file(GLOB CPP_FILES	${PROJECT_SOURCE_DIR}/src/*.cpp
			${PROJECT_SOURCE_DIR}/src/features/*.cpp
			${PROJECT_SOURCE_DIR}/src/filter/*.cpp
			${PROJECT_SOURCE_DIR}/src/images/*.cpp
			${PROJECT_SOURCE_DIR}/src/plot/*.cpp
			${PROJECT_SOURCE_DIR}/src/tools/*.cpp)

file(GLOB HEADER_FILES 	${PROJECT_SOURCE_DIR}/include/*.h
			${PROJECT_SOURCE_DIR}/include/features/*.h
			${PROJECT_SOURCE_DIR}/include/filter/*.h
			${PROJECT_SOURCE_DIR}/include/images/*.h
			${PROJECT_SOURCE_DIR}/include/plot/*.hpp
			${PROJECT_SOURCE_DIR}/include/tools/*.h)

SOURCE_GROUP("Source Files" FILES ${CPP_FILES})
SOURCE_GROUP("Header Files" FILES ${HEADER_FILES})

include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${CPP_FILES} ${HEADER_FILES})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES})

set(${PROJECTNAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                 ${OpenCV_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(${PROJECTNAME}_HEADER_FILES ${HEADER_FILES}
    CACHE INTERNAL "${PROJECT_NAME}: Header files" FORCE)

set(${PROJECTNAME}_CPP_FILES ${CPP_FILES}
    CACHE INTERNAL "${PROJECT_NAME}: Cpp files" FORCE)
