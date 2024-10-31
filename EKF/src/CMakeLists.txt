add_library(ekf
	EKF.cpp
	nav-functions.cpp
	readNavData.cpp
)

target_include_directories(ekf PUBLIC
	"${EKF_standalone_SOURCE_DIR}/include"
)

target_link_libraries(ekf PRIVATE
	Eigen3::Eigen
)

