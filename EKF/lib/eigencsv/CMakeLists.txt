add_library(eigencsv INTERFACE
	eigencsv.h
)

target_include_directories(eigencsv INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(eigencsv INTERFACE
	Eigen3::Eigen
)
