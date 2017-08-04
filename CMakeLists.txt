SET(CMAKE_CXX_FLAGS_DEBUG   "$ENV{CXXFLAGS} -pie -fPIE -O0 -Wall -g -Wall") 
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -pie -fPIE -O3 -Wall    -Wall") 
SET(CMAKE_CXX_FLAGS         "$ENV{CXXFLAGS} -pie -fPIE -O3 -Wall    -Wall") 

SET(CMAKE_C_FLAGS_DEBUG     "$ENV{CFLAGS}   -pie -fPIE -O0 -Wall -g -Werror") 
SET(CMAKE_C_FLAGS_RELEASE   "$ENV{CFLAGS}   -pie -fPIE -O3 -Wall    -Werror") 
SET(CMAKE_C_FLAGS           "$ENV{CFLAGS}   -pie -fPIE -O3 -Wall -g -Werror") 

if(ANDROID)
  add_definitions(-DANDROID)
endif(ANDROID)

