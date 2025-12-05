add_library(3DViewer 3DViewer.cpp)
target_link_libraries(3DViewer external.sdl2 spdlog::spdlog)


add_executable(main main.cpp)
target_link_libraries(main spdlog::spdlog 3DViewer)
