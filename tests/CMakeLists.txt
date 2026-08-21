add_executable(audio_engine_tests
        audio_engine_tests.cpp
)
target_link_libraries(audio_engine_tests PRIVATE audio_engine_core)
if(MSVC)
    target_compile_options(audio_engine_tests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(audio_engine_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wshadow
            -Wold-style-cast -Wcast-align -Wunused -Wformat=2)
endif()
if(MINGW)
    target_link_options(audio_engine_tests PRIVATE
            -static-libgcc -static-libstdc++)
    get_filename_component(TEST_MINGW_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
    add_custom_command(TARGET audio_engine_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${TEST_MINGW_BIN_DIR}/libwinpthread-1.dll"
            "$<TARGET_FILE_DIR:audio_engine_tests>/libwinpthread-1.dll")
endif()
add_test(NAME audio_engine_tests COMMAND audio_engine_tests)

add_executable(profile07_audio_integration_tests
        profile07_audio_integration_tests.cpp
)
target_link_libraries(profile07_audio_integration_tests PRIVATE
        audio_engine_core nlohmann_json::nlohmann_json)
if(WIN32)
    target_link_libraries(profile07_audio_integration_tests PRIVATE avrt)
endif()
target_compile_definitions(profile07_audio_integration_tests PRIVATE
        AUDIO_ENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
if(MSVC)
    target_compile_options(profile07_audio_integration_tests PRIVATE
            /W4 /WX /permissive-)
else()
    target_compile_options(profile07_audio_integration_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wshadow
            -Wold-style-cast -Wcast-align -Wunused -Wformat=2)
endif()
if(MINGW)
    target_link_options(profile07_audio_integration_tests PRIVATE
            -static-libgcc -static-libstdc++)
    add_custom_command(TARGET profile07_audio_integration_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${TEST_MINGW_BIN_DIR}/libwinpthread-1.dll"
            "$<TARGET_FILE_DIR:profile07_audio_integration_tests>/libwinpthread-1.dll")
endif()
add_test(NAME profile07_audio_integration_tests
         COMMAND profile07_audio_integration_tests)
