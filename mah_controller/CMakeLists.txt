add_library(mah_controller MODULE mah_controller.h mah_controller.cpp)
target_link_libraries(mah_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_entities
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
