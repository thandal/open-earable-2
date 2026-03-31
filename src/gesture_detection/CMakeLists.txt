target_sources(app PRIVATE
           ${CMAKE_CURRENT_SOURCE_DIR}/gesture_detection.c
           ${CMAKE_CURRENT_SOURCE_DIR}/gesture_detection_pipeline.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/cooldown_gate_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/local_peak_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/range_gate_stage.cpp
)
