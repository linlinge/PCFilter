   find_package( OpenCV REQUIRED)
AUX_SOURCE_DIRECTORY(. LIB_SRC)
ADD_LIBRARY(LIB ${LIB_SRC})
target_link_libraries( LIB ${OpenCV_LIBS})