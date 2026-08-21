// main.cpp
//
// Application entry point.

#include "plc_emulator/core/application.h"

#include <cstdio>
#include <filesystem>
#include <string>

int main(int argc, char* argv[]) {
  // Resolve fonts, icons and language files from the submitted executable's
  // directory regardless of how Windows launches the program.
  if (argc > 0 && argv[0] && argv[0][0] != '\0') {
    std::error_code path_error;
    const std::filesystem::path executable =
        std::filesystem::absolute(argv[0], path_error);
    if (!path_error && executable.has_parent_path()) {
      std::filesystem::current_path(executable.parent_path(), path_error);
    }
  }
  printf("=====================================\n");
  printf("FX3U PLC Simulator Starting...\n");
  printf("Current: Wiring Mode UI Complete\n");
  printf("Next: Drag & Drop Implementation\n");
  printf("=====================================\n");

  // Report unexpected failures and exit cleanly.
  try {
    bool enable_debug = false;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--Debug" || arg == "--debug") {
        enable_debug = true;
        break;
      }
    }

    plc::Application app(enable_debug);

    if (!app.Initialize()) {
      printf("Failed to initialize application!\n");
      return -1;
    }

    printf("Application initialized successfully!\n");
    printf("Press ESC to exit...\n");

    app.Run();
    app.Shutdown();

    printf("Application terminated normally.\n");
    return 0;
  } catch (const std::exception& e) {
    printf("Exception caught: %s\n", e.what());
    return -1;
  }
}
