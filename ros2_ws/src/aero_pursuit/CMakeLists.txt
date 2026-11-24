cmake_minimum_required(VERSION 3.8)
project(aero_pursuit)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# AERO_SIM=OFF builds only the pursuer side (perception, estimation, control, PX4 offboard,
# gimbal over ROS topics), without Gazebo or GStreamer: the embedded build (Raspberry Pi,
# docker/Dockerfile.pursuer). The simulation side (bridges, scenario runner, spectator
# camera, GCS video) needs the default ON.
option(AERO_SIM "Build the Gazebo/simulation side of the package" ON)
# GCS video streamer (GStreamer): with the simulation by default; also for the Isaac Sim image.
option(AERO_GCS_VIDEO "Build the GCS video streamer (GStreamer)" ${AERO_SIM})

# ==============================================================================
# 1. Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
if(AERO_SIM)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(gz-sim8 REQUIRED)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
endif()
find_package(yaml-cpp REQUIRED)

# Local OpenCV (>= 4.9, first version with cv::TrackerVit). Override with -DOpenCV_DIR=...
# TrackerVit inference uses OpenCV DNN's native engine (its own ONNX
# importer); there is no ONNX Runtime dependency.
if(NOT OpenCV_DIR)
  set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
endif()
find_package(OpenCV 4.9 REQUIRED COMPONENTS core imgproc imgcodecs video dnn)
find_package(PkgConfig REQUIRED)
if(AERO_GCS_VIDEO)
  pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
endif()

# ==============================================================================
# 2. Interfaces
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetTfCmd.msg"
  "msg/PursuitStatus.msg"
  "msg/TargetDetection.msg"
  "msg/TargetBearing.msg"
  "msg/PursuitMetrics.msg"
  "msg/ScenarioDrone.msg"
  "msg/ScenarioStatus.msg"
  "msg/TrackingTruth.msg"
  "msg/SpectatorCommand.msg"
  "srv/ScenarioCommand.srv"
  "srv/SetDrones.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==============================================================================
# 3. Core libraries (ROS independent, testable in isolation)
# ==============================================================================
add_library(${PROJECT_NAME}_estimation SHARED
  src/estimation/constant_velocity_kf.cpp
  src/estimation/singer_kf.cpp
)
target_include_directories(${PROJECT_NAME}_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_estimation Eigen3::Eigen)

# ------------------------------------------------------------------------------
# acados solver of the pursuit MPC (ros2_ws/install_acados.sh; problem in acados/pursuit_ocp.py).
# The C code is generated at configure time with the acados Python venv (acados/generate.py),
# only when the .py files or the acados version change, and compiled into ${PROJECT_NAME}_acados.
# ------------------------------------------------------------------------------
set(ACADOS_ROOT "$ENV{ACADOS_ROOT}" CACHE PATH "acados prefix (ros2_ws/install_acados.sh)")
if(NOT ACADOS_ROOT)
  set(ACADOS_ROOT "$ENV{HOME}/.local/share/acados")
endif()
find_library(ACADOS_LIB acados PATHS ${ACADOS_ROOT}/lib NO_DEFAULT_PATH)
find_library(BLASFEO_LIB blasfeo PATHS ${ACADOS_ROOT}/lib NO_DEFAULT_PATH)
find_library(HPIPM_LIB hpipm PATHS ${ACADOS_ROOT}/lib NO_DEFAULT_PATH)
if(NOT ACADOS_LIB OR NOT EXISTS ${ACADOS_ROOT}/venv/bin/python)
  message(FATAL_ERROR "acados não encontrado em ${ACADOS_ROOT}: rode ros2_ws/install_acados.sh "
    "(ou defina ACADOS_ROOT)")
endif()
set(ACADOS_GEN ${CMAKE_CURRENT_BINARY_DIR}/acados_generated)
file(GLOB ACADOS_PY ${CMAKE_CURRENT_SOURCE_DIR}/acados/*.py)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${ACADOS_PY})  # edit .py => regenerate
set(ACADOS_HASH "")
foreach(f ${ACADOS_PY})
  file(MD5 ${f} h)
  string(APPEND ACADOS_HASH ${h})
endforeach()
file(READ ${ACADOS_ROOT}/.aero_ref ACADOS_VERSION)
# A new acados version (install_acados.sh) also regenerates the solver.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${ACADOS_ROOT}/.aero_ref)
string(APPEND ACADOS_HASH ${ACADOS_VERSION})
if(EXISTS ${ACADOS_GEN}/stamp)
  file(READ ${ACADOS_GEN}/stamp ACADOS_STAMP)
endif()
if(NOT ACADOS_STAMP STREQUAL ACADOS_HASH)
  message(STATUS "acados: gerando o solver do MPC em ${ACADOS_GEN}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ACADOS_SOURCE_DIR=${ACADOS_ROOT}
      ${ACADOS_ROOT}/venv/bin/python -W ignore ${CMAKE_CURRENT_SOURCE_DIR}/acados/generate.py ${ACADOS_GEN}
    RESULT_VARIABLE ACADOS_RESULT OUTPUT_VARIABLE ACADOS_OUTPUT ERROR_VARIABLE ACADOS_ERROR)
  if(NOT ACADOS_RESULT EQUAL 0)
    message(FATAL_ERROR "acados: geração falhou\n${ACADOS_OUTPUT}\n${ACADOS_ERROR}")
  endif()
  file(WRITE ${ACADOS_GEN}/stamp ${ACADOS_HASH})
endif()
# Default (N, dt) variant: its C sources + the function table acados_default_api.cpp.
file(GLOB ACADOS_GEN_SRC
  ${ACADOS_GEN}/*/acados_solver_*.c
  ${ACADOS_GEN}/*/*_model/*.c
  ${ACADOS_GEN}/*/*_cost/*.c
  ${ACADOS_GEN}/*/*_constraints/*.c)
add_library(${PROJECT_NAME}_acados SHARED ${ACADOS_GEN_SRC} ${ACADOS_GEN}/acados_default_api.cpp)
set_source_files_properties(${ACADOS_GEN_SRC} PROPERTIES COMPILE_OPTIONS "-w")  # generated code
target_include_directories(${PROJECT_NAME}_acados PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(ACADOS_INCLUDE_DIRS
  ${ACADOS_GEN} ${ACADOS_ROOT}/include ${ACADOS_ROOT}/include/blasfeo/include
  ${ACADOS_ROOT}/include/hpipm/include)
target_include_directories(${PROJECT_NAME}_acados SYSTEM PUBLIC ${ACADOS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_acados PRIVATE ${ACADOS_LIB} ${HPIPM_LIB} ${BLASFEO_LIB} m)
# libacados has no search path of its own (it needs libhpipm/libblasfeo): DT_RPATH (not RUNPATH)
# on this library covers the whole tree at run time; -rpath-link lets dependents link.
set_target_properties(${PROJECT_NAME}_acados PROPERTIES
  BUILD_RPATH ${ACADOS_ROOT}/lib INSTALL_RPATH ${ACADOS_ROOT}/lib)
target_link_options(${PROJECT_NAME}_acados PRIVATE -Wl,--disable-new-dtags
  INTERFACE -Wl,-rpath-link,${ACADOS_ROOT}/lib)

add_library(${PROJECT_NAME}_control SHARED
  src/control/pursuit_types.cpp
  src/control/pursuit_supervisor.cpp
  src/control/performance_metrics.cpp
  src/control/mpc.cpp
  src/control/acados_variants.cpp
  src/control/gimbal_pitch.cpp
  src/control/course_rate.cpp
)
target_include_directories(${PROJECT_NAME}_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# mpc.cpp calls the acados C interface (ocp_nlp_*) and the default variant of
# ${PROJECT_NAME}_acados (headers PUBLIC there); acados_variants.cpp loads the run-time
# variants with dlopen and generates them with the installed acados/generate.py.
target_link_libraries(${PROJECT_NAME}_control Eigen3::Eigen ${PROJECT_NAME}_acados ${ACADOS_LIB}
  ${CMAKE_DL_LIBS})
string(MD5 ACADOS_HASH_SHORT "${ACADOS_HASH}")
target_compile_definitions(${PROJECT_NAME}_control PRIVATE
  AERO_ACADOS_ROOT="${ACADOS_ROOT}"
  AERO_ACADOS_GENERATOR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/acados/generate.py"
  AERO_ACADOS_HASH="${ACADOS_HASH_SHORT}")
install(DIRECTORY acados/ DESTINATION share/${PROJECT_NAME}/acados FILES_MATCHING PATTERN "*.py")
# Hash part of the variant cache folder (<cache>/<name>_<hash>, control/acados_variants.cpp):
# lets an image pre-build variants where the node cannot generate them (docker/Dockerfile.pursuer).
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/acados_variant_hash ${ACADOS_HASH_SHORT})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/acados_variant_hash DESTINATION share/${PROJECT_NAME}/acados)
# DT_RPATH (not RUNPATH) so the acados of ACADOS_ROOT wins over another one in LD_LIBRARY_PATH.
set_target_properties(${PROJECT_NAME}_control PROPERTIES INSTALL_RPATH ${ACADOS_ROOT}/lib)
target_link_options(${PROJECT_NAME}_control PRIVATE -Wl,--disable-new-dtags)

add_library(${PROJECT_NAME}_evasion SHARED
  src/evasion/evader_policy.cpp
)
target_include_directories(${PROJECT_NAME}_evasion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_evasion Eigen3::Eigen)

# Scenarios: trajectories, plans, swarms and guidance (YAML -> references).
add_library(${PROJECT_NAME}_scenario SHARED
  src/scenario/trajectory.cpp
  src/scenario/trajectory_factory.cpp
  src/scenario/behavior.cpp
  src/scenario/agent_executor.cpp
  src/scenario/guidance.cpp
  src/scenario/scenario_io.cpp
)
target_include_directories(${PROJECT_NAME}_scenario PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_scenario ${PROJECT_NAME}_evasion Eigen3::Eigen yaml-cpp)

# Scenario validation and preview without ROS (used by the GCS).
add_executable(scenario_tool src/tools/scenario_tool.cpp)
target_link_libraries(scenario_tool ${PROJECT_NAME}_scenario)

add_library(${PROJECT_NAME}_perception SHARED
  src/perception/camera_model.cpp
  src/perception/vit_tracker.cpp
  src/perception/track_verifier.cpp
  src/perception/appearance_model.cpp
  src/perception/image_conversions.cpp
  src/perception/overlay.cpp
  src/perception/box_projection.cpp
)
target_include_directories(${PROJECT_NAME}_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_perception SYSTEM BEFORE PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_perception ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_perception sensor_msgs)

# ==============================================================================
# 4. ROS 2 components (loadable in a container; each one also builds an executable)
# ==============================================================================
function(add_component target source)
  add_library(${target} SHARED ${source})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${target} rclcpp rclcpp_components ${ARGN})
endfunction()

add_component(vision_tracker_component src/nodes/vision_tracker_node.cpp
  sensor_msgs geometry_msgs ament_index_cpp)
target_link_libraries(vision_tracker_component
  ${PROJECT_NAME}_perception "${cpp_typesupport_target}")
rclcpp_components_register_node(vision_tracker_component
  PLUGIN "aero_pursuit::VisionTrackerNode" EXECUTABLE vision_tracker_node)

add_component(target_estimator_component src/nodes/target_estimator_node.cpp
  geometry_msgs nav_msgs visualization_msgs px4_msgs tf2 tf2_ros tf2_eigen)
target_link_libraries(target_estimator_component
  ${PROJECT_NAME}_estimation "${cpp_typesupport_target}")
rclcpp_components_register_node(target_estimator_component
  PLUGIN "aero_pursuit::TargetEstimatorNode" EXECUTABLE target_estimator_node)

# Pitch-only camera gimbal (camera_mount:=gimbal): TF of the camera + geometric pointing law.
add_component(gimbal_component src/nodes/gimbal_node.cpp
  geometry_msgs nav_msgs px4_msgs sensor_msgs std_msgs tf2_ros)
target_link_libraries(gimbal_component ${PROJECT_NAME}_control "${cpp_typesupport_target}")
rclcpp_components_register_node(gimbal_component
  PLUGIN "aero_pursuit::GimbalNode" EXECUTABLE gimbal_node)

add_component(pursuit_controller_component src/nodes/pursuit_controller_node.cpp
  geometry_msgs nav_msgs px4_msgs sensor_msgs std_srvs tf2_msgs)
target_sources(pursuit_controller_component PRIVATE src/nodes/pursuit_controller_parameters.cpp)
target_link_libraries(pursuit_controller_component
  ${PROJECT_NAME}_control "${cpp_typesupport_target}")
rclcpp_components_register_node(pursuit_controller_component
  PLUGIN "aero_pursuit::PursuitControllerNode" EXECUTABLE pursuit_controller_node)

add_component(session_recorder_component src/nodes/session_recorder_node.cpp
  rosbag2_cpp rosbag2_storage)
target_sources(session_recorder_component PRIVATE src/nodes/session_recorder_snapshot.cpp)
target_link_libraries(session_recorder_component "${cpp_typesupport_target}" yaml-cpp Eigen3::Eigen)
rclcpp_components_register_node(session_recorder_component
  PLUGIN "aero_pursuit::SessionRecorderNode" EXECUTABLE session_recorder)

add_component(offboard_control_component src/nodes/offboard_control_node.cpp px4_msgs std_msgs)
rclcpp_components_register_node(offboard_control_component
  PLUGIN "aero_pursuit::OffboardControlNode" EXECUTABLE offboard_control)

if(AERO_SIM)  # Gazebo TF bridge
add_component(gazebo_tf_bridge_component src/nodes/gazebo_tf_bridge_node.cpp
  geometry_msgs tf2_ros)
target_link_libraries(gazebo_tf_bridge_component
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
rclcpp_components_register_node(gazebo_tf_bridge_component
  PLUGIN "aero_pursuit::GazeboTfBridgeNode" EXECUTABLE gazebo_tf_bridge)
endif()


# Simulated world seen by the scenario runner: interface + ROS-topic backend (Isaac Sim),
# always built; the Gazebo backend (gz-transport) only with AERO_SIM.
add_library(${PROJECT_NAME}_sim_world SHARED src/sim/world.cpp src/sim/ros_world.cpp)
target_include_directories(${PROJECT_NAME}_sim_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_sim_world Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_sim_world rclcpp geometry_msgs std_msgs tf2_msgs)

# Direct interface with Gazebo (gz-transport): poses, spawning, teleport, twist.
if(AERO_SIM)  # world access
add_library(${PROJECT_NAME}_sim SHARED src/sim/gz_world.cpp)
target_include_directories(${PROJECT_NAME}_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_sim ${PROJECT_NAME}_sim_world Eigen3::Eigen
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)

endif()


# Remote pursuer (another computer): RGB reduced before it crosses the network.
add_component(image_downscaler_component src/nodes/image_downscaler_node.cpp sensor_msgs)
target_link_libraries(image_downscaler_component ${PROJECT_NAME}_perception)
rclcpp_components_register_node(image_downscaler_component
  PLUGIN "aero_pursuit::ImageDownscalerNode" EXECUTABLE image_downscaler)

# Scenario runner: Gazebo (AERO_SIM) or the ROS-topic world (world_backend:=ros, Isaac Sim).
add_component(scenario_runner_component src/nodes/scenario_runner_node.cpp
  geometry_msgs sensor_msgs std_msgs std_srvs visualization_msgs px4_msgs tf2_ros ament_index_cpp)
target_link_libraries(scenario_runner_component
  ${PROJECT_NAME}_scenario ${PROJECT_NAME}_sim_world "${cpp_typesupport_target}")
if(AERO_SIM)
  target_link_libraries(scenario_runner_component ${PROJECT_NAME}_sim)
  target_compile_definitions(scenario_runner_component PRIVATE AERO_HAVE_GZ)
endif()
rclcpp_components_register_node(scenario_runner_component
  PLUGIN "aero_pursuit::ScenarioRunnerNode" EXECUTABLE scenario_runner)

# Free camera of the GCS: Gazebo (AERO_SIM) or the ROS-topic world (world_backend:=ros, Isaac Sim).
add_component(spectator_camera_component src/nodes/spectator_camera_node.cpp
  geometry_msgs tf2_ros ament_index_cpp)
target_link_libraries(spectator_camera_component ${PROJECT_NAME}_sim_world "${cpp_typesupport_target}")
if(AERO_SIM)
  target_link_libraries(spectator_camera_component ${PROJECT_NAME}_sim)
  target_compile_definitions(spectator_camera_component PRIVATE AERO_HAVE_GZ)
endif()
rclcpp_components_register_node(spectator_camera_component
  PLUGIN "aero_pursuit::SpectatorCameraNode" EXECUTABLE spectator_camera)


add_component(target_acquisition_component src/nodes/target_acquisition_node.cpp
  sensor_msgs tf2 tf2_ros tf2_eigen)
target_link_libraries(target_acquisition_component
  ${PROJECT_NAME}_perception "${cpp_typesupport_target}")
rclcpp_components_register_node(target_acquisition_component
  PLUGIN "aero_pursuit::TargetAcquisitionNode" EXECUTABLE target_acquisition)

# Video streaming to the GCS (GStreamer: VA-API / x264 / x265 / JPEG).
if(AERO_GCS_VIDEO)
add_library(${PROJECT_NAME}_streaming SHARED
  src/streaming/stream_config.cpp
  src/streaming/gcs_link.cpp
  src/streaming/video_encoder.cpp)
target_include_directories(${PROJECT_NAME}_streaming PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_streaming PkgConfig::GST)

add_component(video_streamer_component src/nodes/video_streamer_node.cpp sensor_msgs)
# Same custom OpenCV as the perception (before the system includes).
target_include_directories(video_streamer_component SYSTEM BEFORE PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(video_streamer_component
  ${PROJECT_NAME}_streaming opencv_core opencv_imgproc)
rclcpp_components_register_node(video_streamer_component
  PLUGIN "aero_pursuit::VideoStreamerNode" EXECUTABLE video_streamer)
endif()


add_component(gcs_telemetry_component src/nodes/gcs_telemetry_node.cpp tf2_ros tf2_msgs)
rclcpp_components_register_node(gcs_telemetry_component
  PLUGIN "aero_pursuit::GcsTelemetryNode" EXECUTABLE gcs_telemetry)

# Session replay in the simulator: teleports the models to the recorded poses (GCS replay).
add_component(world_replay_component src/nodes/world_replay_node.cpp
  sensor_msgs std_msgs tf2_ros tf2_msgs)
target_link_libraries(world_replay_component ${PROJECT_NAME}_sim_world "${cpp_typesupport_target}")
if(AERO_SIM)
  target_link_libraries(world_replay_component ${PROJECT_NAME}_sim)
  target_compile_definitions(world_replay_component PRIVATE AERO_HAVE_GZ)
endif()
rclcpp_components_register_node(world_replay_component
  PLUGIN "aero_pursuit::WorldReplayNode" EXECUTABLE world_replay)

add_component(joy_teleop_component src/nodes/joy_teleop_node.cpp
  sensor_msgs geometry_msgs std_msgs std_srvs px4_msgs)
rclcpp_components_register_node(joy_teleop_component
  PLUGIN "aero_pursuit::JoyTeleopNode" EXECUTABLE drone_joy_teleop)

# ==============================================================================
# 5. Installation
# ==============================================================================
set(AERO_INSTALL_TARGETS
  image_downscaler_component
  ${PROJECT_NAME}_estimation
  ${PROJECT_NAME}_acados
  ${PROJECT_NAME}_control
  ${PROJECT_NAME}_evasion
  ${PROJECT_NAME}_scenario
  ${PROJECT_NAME}_sim_world
  scenario_runner_component
  ${PROJECT_NAME}_perception
  target_acquisition_component
  gcs_telemetry_component
  vision_tracker_component
  target_estimator_component
  pursuit_controller_component
  gimbal_component
  session_recorder_component
  offboard_control_component
  joy_teleop_component
  spectator_camera_component
  world_replay_component
)
if(AERO_SIM)
  list(APPEND AERO_INSTALL_TARGETS
    ${PROJECT_NAME}_sim
    gazebo_tf_bridge_component)
endif()
if(AERO_GCS_VIDEO)
  list(APPEND AERO_INSTALL_TARGETS ${PROJECT_NAME}_streaming video_streamer_component)
endif()
install(TARGETS ${AERO_INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS scenario_tool DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config gz_models DESTINATION share/${PROJECT_NAME}/)
# Tracker models (download with models/download_models.sh); each one is optional.
foreach(model object_tracking_vittrack_2023sep.onnx nanotrackv2_nanotrack_backbone_sim.onnx
    nanotrackv2_nanotrack_head_sim.onnx dasiamrpn_model.onnx dasiamrpn_kernel_r1.onnx
    dasiamrpn_kernel_cls1.onnx)
  install(FILES models/${model} DESTINATION share/${PROJECT_NAME}/models OPTIONAL)
endforeach()
install(PROGRAMS scripts/run_study.py scripts/analyze_study.py scripts/view_trajectories.py scripts/tune_study.py
  scripts/probe_acquisition.py scripts/calibrate_model_box.py scripts/image_error.py
  DESTINATION lib/${PROJECT_NAME})

# ==============================================================================
# 6. Testes
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_estimation test/test_constant_velocity_kf.cpp test/test_singer_kf.cpp)
  target_link_libraries(test_estimation ${PROJECT_NAME}_estimation)

  ament_add_gtest(test_control test/test_supervisor.cpp test/test_frames.cpp
    test/test_mpc.cpp test/test_gimbal_pitch.cpp)
  target_link_libraries(test_control ${PROJECT_NAME}_control)


  ament_add_gtest(test_scenario test/test_scenario.cpp)
  target_link_libraries(test_scenario ${PROJECT_NAME}_scenario)
  target_compile_definitions(test_scenario PRIVATE
    AERO_PURSUIT_SCENARIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../scenarios")

  ament_add_gtest(test_perception test/test_camera_model.cpp test/test_box_projection.cpp test/test_vit_tracker.cpp
    test/test_track_verifier.cpp test/test_appearance_model.cpp)
  target_link_libraries(test_perception ${PROJECT_NAME}_perception)
  target_compile_definitions(test_perception PRIVATE
    AERO_PURSUIT_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")

  if(AERO_GCS_VIDEO)
    ament_add_gtest(test_streaming test/test_streaming.cpp)
    target_link_libraries(test_streaming ${PROJECT_NAME}_streaming)
  endif()

endif()

ament_package()
