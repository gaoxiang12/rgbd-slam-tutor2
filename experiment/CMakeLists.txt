set( build_libs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    rgbd_tutor_lib
    ${PROJECT_SOURCE_DIR}/Thirdparty/orbslam_modified/lib/libORB_SLAM2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
    ${BOOST_LIBRARIES}
)

add_executable( match_orb match_orbfeature_tum.cpp )
target_link_libraries( match_orb
    ${build_libs}
)

add_executable( run_tracker run_tracker.cpp )
target_link_libraries( run_tracker
   ${build_libs}
)

add_executable( test_pnp test_pnp.cpp )
target_link_libraries( test_pnp
    ${build_libs}
)


add_executable( exp_pose_graph exp_pose_graph.cpp )
target_link_libraries( exp_pose_graph
    ${build_libs}
)

add_executable( exp_mapping exp_mapping.cpp )
target_link_libraries( exp_mapping
    ${build_libs}
)


