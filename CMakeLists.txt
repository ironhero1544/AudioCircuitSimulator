cmake_minimum_required(VERSION 3.20)
project(AudioCircuitSimulator VERSION 0.1.0)

cmake_policy(SET CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Verilator is intentionally not configured by CMake. Only electronic
# component Analyze/Build jobs invoke it through the dedicated WSL2 backend.

option(BUILD_MAIN_APP "Build main PLC simulator app" ON)
option(BUILD_HAPTICS_BRIDGE "Build WinRT haptics bridge DLL" OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks during build" OFF)
option(BUILD_TESTING "Build tests" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Keep the IDE install target writable without administrator privileges.
if(WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install"
            CACHE PATH "Default local install directory" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(WIN32)
    add_compile_definitions(UNICODE _UNICODE)
    if(MSVC)
        add_compile_options(/utf-8)
        add_compile_definitions(_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING)
    else()
        add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
    endif()
endif()

include(FetchContent)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
if(CMAKE_VERSION VERSION_LESS 3.5)
    cmake_policy(VERSION 3.5)
endif()

FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.3.8)
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.89.2)
FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.2)
FetchContent_Declare(miniz GIT_REPOSITORY https://github.com/richgel999/miniz.git GIT_TAG 3.0.2)
FetchContent_Declare(box2d GIT_REPOSITORY https://github.com/erincatto/box2d.git GIT_TAG v3.1.1)

add_library(audio_engine_core STATIC
  src/audio/mna_solver.cpp
  src/audio/compiled_audio_circuit.cpp
  src/audio/converter_models.cpp
)
target_include_directories(audio_engine_core PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
if(MSVC)
    target_compile_options(audio_engine_core PRIVATE
            /W4 /WX /permissive-
            # Realtime DSP must meet its deadline even with the Debug preset.
            # Debug information remains enabled, but /Od is not viable at
            # 48 kHz for the nonlinear MNA solver.
            $<$<CONFIG:Debug>:/O2>)
else()
    target_compile_options(audio_engine_core PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wshadow
            -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
            -Wunused -Woverloaded-virtual -Wformat=2
            -Wimplicit-fallthrough
            # Keep the solver optimized in developer builds. The target still
            # carries -g for stack traces and source-level debugging.
            $<$<CONFIG:Debug>:-O2>)
endif()

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

if(BUILD_MAIN_APP)
    FetchContent_MakeAvailable(glfw imgui nlohmann_json miniz box2d)

    if(TARGET box2d::box2d)
        get_target_property(BOX2D_TARGET box2d::box2d ALIASED_TARGET)
    endif()
    if(NOT BOX2D_TARGET AND TARGET box2d)
        set(BOX2D_TARGET box2d)
    endif()
    if(BOX2D_TARGET)
        if(MSVC)
            target_compile_options(${BOX2D_TARGET} PRIVATE /W0)
        else()
            target_compile_options(${BOX2D_TARGET} PRIVATE -w)
        endif()
        get_target_property(BOX2D_INCLUDES ${BOX2D_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
        if(BOX2D_INCLUDES)
            target_include_directories(${BOX2D_TARGET} SYSTEM INTERFACE ${BOX2D_INCLUDES})
        endif()
    endif()

    configure_file(
            ${glfw_SOURCE_DIR}/deps/glad_gl.c
            ${CMAKE_CURRENT_BINARY_DIR}/generated/glad_gl.c
            COPYONLY
    )

    add_library(glad ${CMAKE_CURRENT_BINARY_DIR}/generated/glad_gl.c)
    target_include_directories(glad PUBLIC ${glfw_SOURCE_DIR}/deps)

    set(IMGUI_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/imgui)
    file(MAKE_DIRECTORY ${IMGUI_GENERATED_DIR})
    foreach(IMGUI_FILE
            imgui.cpp
            imgui_demo.cpp
            imgui_draw.cpp
            imgui_tables.cpp
            imgui_widgets.cpp
            backends/imgui_impl_glfw.cpp
            backends/imgui_impl_opengl3.cpp)
        get_filename_component(IMGUI_SUBDIR ${IMGUI_FILE} DIRECTORY)
        if(IMGUI_SUBDIR)
            file(MAKE_DIRECTORY ${IMGUI_GENERATED_DIR}/${IMGUI_SUBDIR})
        endif()
        configure_file(
                ${imgui_SOURCE_DIR}/${IMGUI_FILE}
                ${IMGUI_GENERATED_DIR}/${IMGUI_FILE}
                COPYONLY
        )
    endforeach()

    add_library(imgui_lib
            ${IMGUI_GENERATED_DIR}/imgui.cpp
            ${IMGUI_GENERATED_DIR}/imgui_demo.cpp
            ${IMGUI_GENERATED_DIR}/imgui_draw.cpp
            ${IMGUI_GENERATED_DIR}/imgui_tables.cpp
            ${IMGUI_GENERATED_DIR}/imgui_widgets.cpp
            ${IMGUI_GENERATED_DIR}/backends/imgui_impl_glfw.cpp
            ${IMGUI_GENERATED_DIR}/backends/imgui_impl_opengl3.cpp
    )

    target_include_directories(imgui_lib PUBLIC
            ${imgui_SOURCE_DIR}
            ${imgui_SOURCE_DIR}/backends
            ${glfw_SOURCE_DIR}/include
    )

    find_package(OpenGL REQUIRED)

    add_executable(PLCSimulator
            src/main.cpp

            src/core/application.cpp
            src/application/app_layout.cpp
            src/application/app_lod.cpp
            src/application/app_input.cpp
            src/application/app_debug.cpp
            src/application/app_render.cpp
            src/application/app_project.cpp
            src/application/app_rtl.cpp
            src/application/app_lifecycle.cpp
            src/core/win32_input.cpp
            src/core/win32_haptics_stub.cpp
             src/core/ui_settings.cpp
             src/core/windows_power_utils.cpp
             src/core/data_types.cpp
             src/core/plc_simulator_core.cpp
             src/core/window_manager.cpp
            src/core/render_manager.cpp
            src/core/input_handler.cpp
            src/core/component_manager.cpp
            src/input/platform_input_collector.cpp
            src/input/ui_capture_registry.cpp

            src/components/component_registry.cpp
            src/components/audio_components_def.cpp
            src/audio/audio_circuit_runtime.cpp
            src/audio/audio_stream_bridge.cpp
            src/audio/virtual_audio_installer.cpp
            src/components/component_behavior.cpp
            src/components/button_unit_def.cpp
            src/components/box_def.cpp
            src/components/conveyor_def.cpp
            src/components/cylinder_def.cpp
            src/components/emergency_stop_def.cpp
            src/components/frl_def.cpp
            src/components/inductive_sensor_def.cpp
            src/components/limit_switch_def.cpp
            src/components/manifold_def.cpp
            src/components/meter_valve_def.cpp
            src/components/plc_def.cpp
            src/components/processing_cylinder_def.cpp
            src/components/power_supply_def.cpp
            src/components/ring_sensor_def.cpp
            src/components/sensor_def.cpp
            src/components/tower_lamp_def.cpp
            src/components/valve_double_def.cpp
            src/components/valve_single_def.cpp
            src/components/workpiece_def.cpp
            src/components/rtl_module_def.cpp
            src/third_party/nanosvg_impl.cpp

            src/lang/lang_manager.cpp

            src/wiring/application_components.cpp
            src/wiring/application_wiring.cpp
            src/wiring/application_ports.cpp
            src/wiring/application_snap.cpp

            src/programming/programming_mode.cpp
            src/programming/programming_mode_sim.cpp
            src/programming/programming_mode_edit.cpp
            src/programming/programming_mode_ui.cpp
            src/programming/compiled_plc_executor.cpp

            src/physics/physics_engine.cpp
            src/physics/component_physics_adapter.cpp
            src/physics/physics_solvers.cpp
            src/physics/application_physics.cpp
            src/application_physics/physics_task_runner.cpp
            src/application_physics/physics_update.cpp
            src/application_physics/physics_box2d.cpp
            src/application_physics/physics_electrical.cpp
            src/application_physics/physics_pneumatic.cpp
            src/application_physics/physics_actuators.cpp
            src/application_physics/physics_workpiece.cpp
            src/application_physics/physics_ports.cpp
            src/application_physics/physics_plc.cpp
            src/application_physics/physics_sync.cpp
            src/application_physics/physics_stats.cpp

            src/io/io_mapper.cpp

            src/rtl/rtl_project_manager.cpp
            src/rtl/rtl_runtime_manager.cpp
            src/rtl/audio_rtl_wsl_toolchain.cpp

            src/data/data_repository.cpp
            src/data/event_system.cpp
            src/data/memory_manager.cpp

            src/project/project_file_manager.cpp
            src/project/xml_serializer.cpp
            src/project/ladder_to_ld_converter.cpp
            src/project/ld_to_ladder_converter.cpp
            src/project/ladder_ir.cpp
            src/project/openplc_compiler_integration.cpp
    )

    if(WIN32)
        target_sources(PLCSimulator PRIVATE resources/icon/app_icon.rc)
    endif()

    target_include_directories(PLCSimulator PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${glfw_SOURCE_DIR}/deps
            ${glfw_SOURCE_DIR}/include
            ${nlohmann_json_SOURCE_DIR}/include
            ${miniz_SOURCE_DIR}
    )

    target_include_directories(PLCSimulator SYSTEM PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party
            ${imgui_SOURCE_DIR}
            ${imgui_SOURCE_DIR}/backends
    )

    set(FONT_DIR ${CMAKE_SOURCE_DIR}/resources/fonts)
    if(EXISTS ${FONT_DIR})
        add_custom_command(
                TARGET PLCSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/fonts"
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${FONT_DIR}"
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/fonts"
        )
    endif()

    set(LANG_DIR ${CMAKE_SOURCE_DIR}/resources/lang)
    if(EXISTS ${LANG_DIR})
        add_custom_command(
                TARGET PLCSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/lang"
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${LANG_DIR}"
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/lang"
        )
    endif()

    set(ICON_DIR ${CMAKE_SOURCE_DIR}/resources/icon)
    if(EXISTS ${ICON_DIR})
        add_custom_command(
                TARGET PLCSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/icon"
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ICON_DIR}"
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/icon"
        )
    endif()

    set(SYMBOL_DIR ${CMAKE_SOURCE_DIR}/resources/symbol)
    if(EXISTS ${SYMBOL_DIR})
        add_custom_command(
                TARGET PLCSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/symbol"
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${SYMBOL_DIR}"
                "$<TARGET_FILE_DIR:PLCSimulator>/resources/symbol"
        )
    endif()

    # These translation units execute on the 48 kHz PCM/RTL path. Keeping
    # them at the UI target's Debug -O0 setting makes the FFT, block plumbing,
    # and DAC worker miss deadlines even though the MNA library is optimized.
    if(MSVC)
        set_property(SOURCE
                src/audio/audio_circuit_runtime.cpp
                src/audio/audio_stream_bridge.cpp
                src/rtl/rtl_runtime_manager.cpp
                APPEND PROPERTY COMPILE_OPTIONS $<$<CONFIG:Debug>:/O2>)
    else()
        set_property(SOURCE
                src/audio/audio_circuit_runtime.cpp
                src/audio/audio_stream_bridge.cpp
                src/rtl/rtl_runtime_manager.cpp
                APPEND PROPERTY COMPILE_OPTIONS $<$<CONFIG:Debug>:-O2>)
    endif()

    target_link_libraries(PLCSimulator PRIVATE
            audio_engine_core
            glfw
            imgui_lib
            glad
            miniz
            box2d
            OpenGL::GL
            comdlg32
            shell32
            winmm
            avrt
            ole32
            uuid
    )

    set_target_properties(PLCSimulator PROPERTIES OUTPUT_NAME AudioCircuitSimulator)

    if(MSVC)
        target_compile_options(PLCSimulator PRIVATE
                /W4 /permissive- /Zc:__cplusplus /Zc:inline /utf-8
                $<$<CONFIG:Release>:/O2>
                $<$<CONFIG:Release>:/GL>
                $<$<CONFIG:Debug>:/Od>
                $<$<CONFIG:Debug>:/Zi>
                $<$<CONFIG:Debug>:/RTC1>
        )

        target_compile_definitions(PLCSimulator PRIVATE
                _CRT_SECURE_NO_WARNINGS
                NOMINMAX
                WIN32_LEAN_AND_MEAN
                _UNICODE
                UNICODE
                $<$<CONFIG:Debug>:_DEBUG>
                $<$<CONFIG:Release>:NDEBUG>
        )

        target_link_options(PLCSimulator PRIVATE
                $<$<CONFIG:Release>:/LTCG>
                $<$<CONFIG:Release>:/OPT:REF>
                $<$<CONFIG:Release>:/OPT:ICF>
                # Release is a GUI subsystem application, but keep the existing
                # standard main(argc, argv) entry point. mainCRTStartup performs
                # normal CRT initialization and then calls main without creating
                # a console window.
                $<$<CONFIG:Release>:/SUBSYSTEM:WINDOWS>
                $<$<CONFIG:Release>:/ENTRY:mainCRTStartup>
        )
    else()
        target_compile_options(PLCSimulator PRIVATE
                -Wall -Wextra -Wpedantic -Werror
                -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
                -Wunused -Woverloaded-virtual -Wformat=2 -Wimplicit-fallthrough
                $<$<CONFIG:Release>:-O3>
                $<$<CONFIG:Release>:-DNDEBUG>
                $<$<CONFIG:Release>:-Wno-array-bounds>
                $<$<CONFIG:Release>:-Wno-stringop-overflow>
                $<$<CONFIG:Debug>:-O0>
                $<$<CONFIG:Debug>:-g3>
                $<$<CONFIG:Debug>:-D_DEBUG>
        )

        target_link_options(PLCSimulator PRIVATE
                -static-libgcc
                -static-libstdc++
        )
        # Keep winpthreads dynamic. Newer MinGW-w64 distributions (including
        # CLion 2026.2's GCC 15 toolchain) ship a static libwinpthread archive
        # that relies on CRT-private setjmp intrinsics and cannot be mixed with
        # this partially static executable. The runtime DLL is already bundled
        # below, so forcing -Bstatic here was both inconsistent and fragile.
        get_filename_component(AUDIO_SIM_MINGW_BIN_DIR
                               "${CMAKE_CXX_COMPILER}" DIRECTORY)
        get_filename_component(AUDIO_SIM_MINGW_ROOT_DIR
                               "${AUDIO_SIM_MINGW_BIN_DIR}" DIRECTORY)
        file(GLOB AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARIES
                "${AUDIO_SIM_MINGW_ROOT_DIR}/*/lib/libwinpthread.dll.a")
        list(LENGTH AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARIES
                    AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARY_COUNT)
        if(AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARY_COUNT EQUAL 0)
            message(FATAL_ERROR
                    "MinGW winpthreads import library was not found under ${AUDIO_SIM_MINGW_ROOT_DIR}")
        endif()
        list(GET AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARIES 0
                 AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARY)
        target_link_libraries(PLCSimulator PRIVATE
                "${AUDIO_SIM_WINPTHREAD_IMPORT_LIBRARY}"
        )
        add_custom_command(TARGET PLCSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${AUDIO_SIM_MINGW_BIN_DIR}/libwinpthread-1.dll"
                        "$<TARGET_FILE_DIR:PLCSimulator>/libwinpthread-1.dll"
                COMMENT "Bundling the MinGW pthread runtime")
        add_custom_command(TARGET PLCSimulator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                        "${CMAKE_SOURCE_DIR}/resources"
                        "$<TARGET_FILE_DIR:PLCSimulator>/resources"
                COMMENT "Bundling the original UI resources")

        set_source_files_properties(src/core/win32_input.cpp PROPERTIES
                COMPILE_OPTIONS "-Wno-cast-function-type")
    endif()
endif()

if(WIN32 AND BUILD_HAPTICS_BRIDGE)
    add_subdirectory(haptics_bridge)
endif()

# =============================================================================
# Testing
# =============================================================================

if(BUILD_TESTING)
    if(NOT TARGET nlohmann_json::nlohmann_json)
        FetchContent_MakeAvailable(nlohmann_json)
    endif()
    enable_testing()
    add_subdirectory(tests)
endif()

# =============================================================================
# Install / Packaging
# =============================================================================

if(BUILD_MAIN_APP)
    install(TARGETS PLCSimulator
            RUNTIME DESTINATION .)

    # Bundle required MinGW runtime DLLs for portable install/installer output.
    if(MINGW)
        get_filename_component(MINGW_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
        set(REQUIRED_MINGW_DLLS
                libgcc_s_seh-1.dll
                libstdc++-6.dll
                libwinpthread-1.dll
        )

        foreach(RUNTIME_DLL IN LISTS REQUIRED_MINGW_DLLS)
            set(RUNTIME_DLL_PATH "${MINGW_BIN_DIR}/${RUNTIME_DLL}")
            if(EXISTS "${RUNTIME_DLL_PATH}")
                install(FILES "${RUNTIME_DLL_PATH}"
                        DESTINATION ".")
            else()
                message(WARNING "Required MinGW runtime DLL not found: ${RUNTIME_DLL_PATH}")
            endif()
        endforeach()
    endif()

    if(TARGET haptics_bridge)
        install(TARGETS haptics_bridge
                RUNTIME DESTINATION .
                LIBRARY DESTINATION .
                ARCHIVE DESTINATION .)
    endif()

    if(EXISTS "${CMAKE_SOURCE_DIR}/resources")
        install(DIRECTORY "${CMAKE_SOURCE_DIR}/resources/"
                DESTINATION "resources")
    endif()

    # Ship the self-contained project packages used by the tutorial and fault
    # demonstrations. The unpacked generator/test directories are intentionally
    # omitted; users only need the loadable .acproj files and their guide.
    file(GLOB BUNDLED_AUDIO_EXAMPLES CONFIGURE_DEPENDS
            "${CMAKE_SOURCE_DIR}/examples/*.acproj")
    if(BUNDLED_AUDIO_EXAMPLES)
        install(FILES ${BUNDLED_AUDIO_EXAMPLES}
                DESTINATION "examples")
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/examples/README.md")
        install(FILES "${CMAKE_SOURCE_DIR}/examples/README.md"
                DESTINATION "examples")
    endif()

    if(EXISTS "${CMAKE_SOURCE_DIR}/README.md")
        install(FILES "${CMAKE_SOURCE_DIR}/README.md"
                DESTINATION ".")
    endif()

    if(EXISTS "${CMAKE_SOURCE_DIR}/README_kr.md")
        install(FILES "${CMAKE_SOURCE_DIR}/README_kr.md"
                DESTINATION ".")
    endif()

    if(EXISTS "${CMAKE_SOURCE_DIR}/README_ja.md")
        install(FILES "${CMAKE_SOURCE_DIR}/README_ja.md"
                DESTINATION ".")
    endif()

    set(PLC_LICENSE_FILE "")
    if(EXISTS "${CMAKE_SOURCE_DIR}/LICENSE-GPL-3.0.txt")
        set(PLC_LICENSE_FILE "${CMAKE_SOURCE_DIR}/LICENSE-GPL-3.0.txt")
    elseif(EXISTS "${CMAKE_SOURCE_DIR}/LICENSE")
        set(PLC_LICENSE_FILE "${CMAKE_SOURCE_DIR}/LICENSE")
    endif()

    if(PLC_LICENSE_FILE)
        install(FILES "${PLC_LICENSE_FILE}"
                DESTINATION ".")
    endif()

    # =============================================================================
    # CPack
    # =============================================================================

    set(CPACK_PACKAGE_NAME "AudioCircuitSimulator")
    set(CPACK_PACKAGE_VENDOR "ironhero")
    set(CPACK_PACKAGE_CONTACT "ironhero1544@users.noreply.github.com")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
            "PLC wiring/programming simulator with custom physics engine")

    set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
    set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

    set(CPACK_PACKAGE_INSTALL_DIRECTORY "AudioCircuitSimulator")
    set(CPACK_MONOLITHIC_INSTALL ON)

    if(PLC_LICENSE_FILE)
        set(CPACK_RESOURCE_FILE_LICENSE "${PLC_LICENSE_FILE}")
    endif()

    if(WIN32)
        set(CPACK_GENERATOR "NSIS;ZIP")
        set(CPACK_PACKAGE_EXECUTABLES "AudioCircuitSimulator" "Audio Circuit Simulator")
        set(CPACK_CREATE_DESKTOP_LINKS "AudioCircuitSimulator")

        set(CPACK_NSIS_DISPLAY_NAME "Audio Circuit Simulator")
        set(CPACK_NSIS_PACKAGE_NAME "AudioCircuitSimulator")
        set(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
        set(CPACK_NSIS_URL_INFO_ABOUT "https://github.com/ironhero1544")
        set(CPACK_NSIS_HELP_LINK "https://github.com/ironhero1544")
        set(CPACK_NSIS_EXECUTABLES_DIRECTORY ".")
        set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
        set(CPACK_NSIS_MODIFY_PATH ON)
        # Optional cleanup of Verilator only. WSL/Ubuntu and generic build tools
        # are preserved because they may be shared by other user workflows.
        set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS
            "MessageBox MB_YESNO|MB_ICONQUESTION \\\"Remove Verilator from the default WSL distribution? WSL and Ubuntu will be kept.\\\" IDNO +2\n"
            "ExecWait '\\\"wsl.exe\\\" --user root --exec /usr/bin/apt-get remove -y verilator' $0\n")

        if(EXISTS "${CMAKE_SOURCE_DIR}/resources/icon/icon.ico")
            set(CPACK_NSIS_MUI_ICON "${CMAKE_SOURCE_DIR}/resources/icon/icon.ico")
            set(CPACK_NSIS_MUI_UNIICON "${CMAKE_SOURCE_DIR}/resources/icon/icon.ico")
            set(CPACK_NSIS_INSTALLED_ICON_NAME "AudioCircuitSimulator.exe")
            set(CPACK_NSIS_MUI_FINISHPAGE_RUN "AudioCircuitSimulator.exe")
        endif()
    else()
        set(CPACK_GENERATOR "TGZ")
    endif()

    include(CPack)
endif()

message(STATUS "=============================================================================")
message(STATUS "PLC Simulator Configuration Summary")
message(STATUS "=============================================================================")
message(STATUS "  Build main app:       ${BUILD_MAIN_APP}")
message(STATUS "  Build haptics bridge: ${BUILD_HAPTICS_BRIDGE}")
message(STATUS "  Compiler:             ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Compiler path:        ${CMAKE_CXX_COMPILER}")
message(STATUS "  Version:              ${PROJECT_VERSION}")
message(STATUS "=============================================================================")
