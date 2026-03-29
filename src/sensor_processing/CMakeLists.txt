target_sources(app PRIVATE
           ${CMAKE_CURRENT_SOURCE_DIR}/sensor_processing_consumer.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/sensor_processing_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/processing_pipeline.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/sensor_source_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/zero_crossing_detector_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/biquad_filter_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/sensor_component_extractor.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/peak_detector_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/switch_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/processing_utils.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/adding_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/moving_mean_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/multiply_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/and_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/not_stage.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/if_stage.cpp
)
