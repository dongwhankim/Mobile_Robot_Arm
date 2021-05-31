# 요구 CMake 최소 버전
CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
# 프로젝트 이름 및 버전
PROJECT ( "kist" )
 

# 빌드 형상(Configuration) 및 주절주절 Makefile 생성 여부 & # 공통 컴파일러

set(CMAKE_CXX_COMPILER g++)
#set(CMAKE_CXX_FLAGS "-m64")
set(THREADS_PREFER_PTHREAD_FLAG ON)
add_definitions("-Wall -std=c++14")
 
# 공통 헤더 파일 Include 디렉토리 (-I)
INCLUDE_DIRECTORIES ( include /usr/local/include/eigen3 /lib/x86_64-linux-gnu )
 
# 공통 컴파일 옵션, 링크 옵션
ADD_COMPILE_OPTIONS ( -g -Wall )
SET ( CMAKE_EXE_LINKER_FLAGS "-static -Wl,--gc-sections" )
add_subdirectory(src)
 
# "Debug" 형상 한정 컴파일 옵션, 링크 옵션
#SET ( CMAKE_C_FLAGS_DEBUG "-DDEBUG -DC_FLAGS" )
#SET ( CMAKE_EXE_LINKER_FLAGS_DEBUG "-DDEBUG -DLINKER_FLAGS" )
 
# "Release" 형상 한정 컴파일 옵션, 링크 옵션
#SET ( CMAKE_C_FLAGS_RELEASE "-DRELEASE -DC_FLAGS" )
#SET ( CMAKE_EXE_LINKER_FLAGS_RELEASE "-DRELEASE -DLINKER_FLAGS" )
 
# 출력 디렉토리
SET ( CMAKE_RUNTIME_OUTPUT_DIRECTORY /build )
SET ( CMAKE_LIBRARY_OUTPUT_DIRECTORY /build )
SET ( CMAKE_ARCHIVE_OUTPUT_DIRECTORY /build )

set(THREADS_PREFER_PTHREAD_FLAG ON)
