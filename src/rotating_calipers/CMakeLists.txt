add_library(rotating_calipers SHARED
            RotatingCalipers.cpp
            RotatingCalipers.h)

target_link_libraries(rotating_calipers ${OpenCV_LIBS})

install(TARGETS
    rotating_calipers
    DESTINATION lib/
    )