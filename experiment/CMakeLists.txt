add_executable( helloslam helloslam.cpp )

add_executable( reading_frame reading_frame.cpp )
target_link_libraries( reading_frame rgbd_tutor ${thirdparty_libs} )
