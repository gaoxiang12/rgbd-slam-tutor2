add_library( rgbd_tutor_lib
    rgbdframe.cpp parameter_reader.cpp orb.cpp pnp.cpp track.cpp looper.cpp pose_graph.cpp
    mapper.cpp
)

target_link_libraries( rgbd_tutor_lib
    ${OpenCV_LIBRARIES}
    ${CHOLMOD_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/Thirdparty/orbslam_modified/lib/libORB_SLAM2.so
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
    )

