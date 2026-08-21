#include "plc_emulator/rtl/audio_rtl_wsl_toolchain.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#endif

namespace plc {
namespace {

std::wstring QuoteWindowsArgument(const std::wstring& argument) {
  if (argument.find_first_of(L" \t\n\v\"") == std::wstring::npos) {
    return argument;
  }
  std::wstring result = L"\"";
  std::size_t backslashes = 0;
  for (wchar_t character : argument) {
    if (character == L'\\') {
      ++backslashes;
      continue;
    }
    if (character == L'\"') {
      result.append(backslashes * 2 + 1, L'\\');
      result.push_back(L'\"');
    } else {
      result.append(backslashes, L'\\');
      result.push_back(character);
    }
    backslashes = 0;
  }
  result.append(backslashes * 2, L'\\');
  result.push_back(L'\"');
  return result;
}

std::string WideToUtf8(const std::wstring& text) {
#ifdef _WIN32
  if (text.empty()) return {};
  int size = WideCharToMultiByte(CP_UTF8, 0, text.data(),
                                 static_cast<int>(text.size()), nullptr, 0,
                                 nullptr, nullptr);
  if (size <= 0) return {};
  std::string result(static_cast<std::size_t>(size), '\0');
  WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
                      result.data(), size, nullptr, nullptr);
  return result;
#else
  return std::string(text.begin(), text.end());
#endif
}

std::vector<std::wstring> BuildArguments(const AudioRtlWslCommand& command) {
  std::vector<std::wstring> arguments;
  if (command.distribution && !command.distribution->empty()) {
    arguments.emplace_back(L"--distribution");
    arguments.push_back(*command.distribution);
  }
  if (command.run_as_root) {
    arguments.emplace_back(L"--user");
    arguments.emplace_back(L"root");
  }
  arguments.emplace_back(L"--cd");
  arguments.push_back(command.working_directory.value_or(L"~"));
  arguments.emplace_back(L"--exec");
  if (!command.environment.empty()) {
    arguments.emplace_back(L"/usr/bin/env");
    for (const auto& [name, value] : command.environment) {
      arguments.push_back(name + L"=" + value);
    }
  }
  arguments.push_back(command.program);
  arguments.insert(arguments.end(), command.arguments.begin(),
                   command.arguments.end());
  return arguments;
}

AudioRtlProcessResult Probe(const std::wstring& program,
                            std::vector<std::wstring> arguments = {}) {
  AudioRtlWslCommand command;
  command.program = program;
  command.arguments = std::move(arguments);
  return AudioRtlWslToolchain::Run(command, 15000);
}

std::string Trim(std::string text) {
  while (!text.empty() &&
         std::isspace(static_cast<unsigned char>(text.back())) != 0) {
    text.pop_back();
  }
  std::size_t start = 0;
  while (start < text.size() &&
         std::isspace(static_cast<unsigned char>(text[start])) != 0) {
    ++start;
  }
  return text.substr(start);
}

#ifdef _WIN32
AudioRtlProcessResult RunElevatedWslHostCommand(
    const std::vector<std::wstring>& arguments, std::uint32_t timeout_ms) {
  AudioRtlProcessResult result;

  std::wstring parameters;
  for (const auto& argument : arguments) {
    if (!parameters.empty()) parameters.push_back(L' ');
    parameters += QuoteWindowsArgument(argument);
  }

  SHELLEXECUTEINFOW execute{};
  execute.cbSize = sizeof(execute);
  execute.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_FLAG_NO_UI;
  execute.lpVerb = L"runas";
  execute.lpFile = L"wsl.exe";
  execute.lpParameters = parameters.c_str();
  execute.nShow = SW_HIDE;

  if (!ShellExecuteExW(&execute) || execute.hProcess == nullptr) {
    const DWORD error = GetLastError();
    result.error = error == ERROR_CANCELLED
                       ? "WSL installation was cancelled at the administrator prompt."
                       : "Failed to start the WSL installer. Windows error " +
                             std::to_string(error) + ".";
    return result;
  }

  result.started = true;
  const DWORD wait = WaitForSingleObject(execute.hProcess, timeout_ms);
  if (wait == WAIT_TIMEOUT) {
    result.timed_out = true;
    result.error = "WSL installation timed out.";
    CloseHandle(execute.hProcess);
    return result;
  }
  if (wait == WAIT_FAILED) {
    result.error = "Failed while waiting for the WSL installer. Windows error " +
                   std::to_string(GetLastError()) + ".";
    CloseHandle(execute.hProcess);
    return result;
  }

  DWORD exit_code = 1;
  if (!GetExitCodeProcess(execute.hProcess, &exit_code)) {
    result.error = "Cannot read the WSL installer exit code. Windows error " +
                   std::to_string(GetLastError()) + ".";
  }
  result.exit_code = exit_code;
  CloseHandle(execute.hProcess);

  if (exit_code != 0 && result.error.empty()) {
    result.error = "wsl.exe --install exited with code " +
                   std::to_string(exit_code) + ".";
  }
  return result;
}
#endif

}  // namespace

AudioRtlProcessResult AudioRtlWslToolchain::Run(
    const AudioRtlWslCommand& command, std::uint32_t timeout_ms) {
  AudioRtlProcessResult result;
  if (command.program.empty()) {
    result.error = "WSL program is empty.";
    return result;
  }
#ifdef _WIN32
  SECURITY_ATTRIBUTES security{};
  security.nLength = sizeof(security);
  security.bInheritHandle = TRUE;
  HANDLE read_pipe = nullptr;
  HANDLE write_pipe = nullptr;
  if (!CreatePipe(&read_pipe, &write_pipe, &security, 0)) {
    result.error = "Failed to create WSL output pipe.";
    return result;
  }
  SetHandleInformation(read_pipe, HANDLE_FLAG_INHERIT, 0);
  HANDLE null_input = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ,
                                  &security, OPEN_EXISTING, 0, nullptr);
  STARTUPINFOW startup{};
  startup.cb = sizeof(startup);
  startup.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
  startup.wShowWindow = SW_HIDE;
  startup.hStdInput = null_input;
  startup.hStdOutput = write_pipe;
  startup.hStdError = write_pipe;

  std::wstring command_line = L"wsl.exe";
  for (const auto& argument : BuildArguments(command)) {
    command_line.push_back(L' ');
    command_line += QuoteWindowsArgument(argument);
  }
  std::vector<wchar_t> mutable_command(command_line.begin(),
                                       command_line.end());
  mutable_command.push_back(L'\0');
  PROCESS_INFORMATION process{};
  HANDLE job = CreateJobObjectW(nullptr, nullptr);
  if (job) {
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags =
        JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    SetInformationJobObject(job, JobObjectExtendedLimitInformation, &limits,
                            sizeof(limits));
  }
  BOOL created = CreateProcessW(nullptr, mutable_command.data(), nullptr,
                                nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                                nullptr, &startup, &process);
  CloseHandle(write_pipe);
  if (null_input != INVALID_HANDLE_VALUE) CloseHandle(null_input);
  if (!created) {
    result.error = "Failed to start wsl.exe. Windows error " +
                   std::to_string(GetLastError()) + ".";
    CloseHandle(read_pipe);
    if (job) CloseHandle(job);
    return result;
  }
  result.started = true;
  if (job) AssignProcessToJobObject(job, process.hProcess);
  const DWORD start_tick = GetTickCount();
  for (;;) {
    DWORD available = 0;
    while (PeekNamedPipe(read_pipe, nullptr, 0, nullptr, &available, nullptr) &&
           available > 0) {
      std::vector<char> buffer(std::min<DWORD>(available, 8192));
      DWORD read = 0;
      if (!ReadFile(read_pipe, buffer.data(),
                    static_cast<DWORD>(buffer.size()), &read, nullptr) ||
          read == 0) {
        break;
      }
      result.output.append(buffer.data(), read);
    }
    if (WaitForSingleObject(process.hProcess, 20) != WAIT_TIMEOUT) break;
    if (GetTickCount() - start_tick > timeout_ms) {
      result.timed_out = true;
      if (job) TerminateJobObject(job, ERROR_TIMEOUT);
      else TerminateProcess(process.hProcess, ERROR_TIMEOUT);
      WaitForSingleObject(process.hProcess, 2000);
      break;
    }
  }
  for (;;) {
    char buffer[8192];
    DWORD available = 0;
    DWORD read = 0;
    if (!PeekNamedPipe(read_pipe, nullptr, 0, nullptr, &available, nullptr) ||
        available == 0 ||
        !ReadFile(read_pipe, buffer, std::min<DWORD>(available, sizeof(buffer)),
                  &read, nullptr) ||
        read == 0) {
      break;
    }
    result.output.append(buffer, read);
  }
  DWORD exit_code = 1;
  GetExitCodeProcess(process.hProcess, &exit_code);
  result.exit_code = exit_code;
  CloseHandle(process.hThread);
  CloseHandle(process.hProcess);
  CloseHandle(read_pipe);
  if (job) CloseHandle(job);
  result.output = Trim(result.output);
  if (result.timed_out) result.error = "WSL command timed out.";
#else
  (void)timeout_ms;
  result.error = "WSL toolchain is available only on Windows.";
#endif
  return result;
}

AudioRtlWslStatus AudioRtlWslToolchain::Detect() {
  AudioRtlWslStatus status;
  AudioRtlProcessResult uname = Probe(L"/usr/bin/uname", {L"-a"});
  status.wsl_found = uname.Succeeded();
  if (!status.wsl_found) {
    status.diagnosis = uname.error.empty() ? uname.output : uname.error;
    if (status.diagnosis.empty()) status.diagnosis = "WSL2 is not ready.";
    return status;
  }
  AudioRtlProcessResult verilator = Probe(L"/usr/bin/verilator", {L"--version"});
  AudioRtlProcessResult compiler = Probe(L"/usr/bin/g++", {L"--version"});
  AudioRtlProcessResult make = Probe(L"/usr/bin/make", {L"--version"});
  status.verilator_found = verilator.Succeeded();
  status.compiler_found = compiler.Succeeded();
  status.make_found = make.Succeeded();
  status.verilator_version = verilator.output;
  status.diagnosis = status.verilator_found && status.compiler_found && status.make_found
                         ? "WSL Verilator component backend is ready."
                         : "Install verilator, g++, and make in the default WSL distribution.";
  return status;
}

AudioRtlProcessResult AudioRtlWslToolchain::Install() {
  // Preserve the existing fast path. On machines that already have a usable
  // default WSL distribution, installation behaves exactly as before.
  AudioRtlProcessResult ready = Probe(L"/usr/bin/uname", {L"-a"});
  if (!ready.Succeeded()) {
#ifdef _WIN32
    // A fresh Windows installation may have wsl.exe but no enabled WSL
    // platform/distribution yet. Bootstrap the standard Ubuntu distribution
    // directly through wsl.exe; no PowerShell script is involved.
    AudioRtlProcessResult bootstrap = RunElevatedWslHostCommand(
        {L"--install", L"-d", L"Ubuntu", L"--no-launch"},
        30 * 60 * 1000);
    if (!bootstrap.Succeeded()) return bootstrap;

    // If Windows enabled WSL components for the first time, a reboot can be
    // required before Linux processes can start. Do not change the existing
    // default-distribution behavior; simply ask the user to retry afterwards.
    ready = Probe(L"/usr/bin/uname", {L"-a"});
    if (!ready.Succeeded()) {
      AudioRtlProcessResult pending = bootstrap;
      pending.exit_code = 1;
      pending.output =
          "WSL/Ubuntu installation completed, but the default WSL "
          "distribution is not ready yet. Restart Windows if requested, "
          "then click Install Tools again.";
      pending.error = pending.output;
      return pending;
    }
#else
    return ready;
#endif
  }

  AudioRtlWslCommand update;
  update.run_as_root = true;
  update.program = L"/usr/bin/apt-get";
  update.arguments = {L"update"};
  AudioRtlProcessResult result = Run(update, 10 * 60 * 1000);
  if (!result.Succeeded()) return result;
  AudioRtlWslCommand install = update;
  install.arguments = {L"install", L"-y", L"verilator", L"g++", L"make"};
  AudioRtlProcessResult installed = Run(install, 20 * 60 * 1000);
  installed.output = result.output + "\n" + installed.output;
  return installed;
}

AudioRtlProcessResult AudioRtlWslToolchain::InstallVerilator() {
  AudioRtlProcessResult ready = Probe(L"/usr/bin/uname", {L"-a"});
  if (!ready.Succeeded()) {
#ifdef _WIN32
    AudioRtlProcessResult bootstrap = RunElevatedWslHostCommand(
        {L"--install", L"-d", L"Ubuntu", L"--no-launch"},
        30 * 60 * 1000);
    if (!bootstrap.Succeeded()) return bootstrap;

    ready = Probe(L"/usr/bin/uname", {L"-a"});
    if (!ready.Succeeded()) {
      AudioRtlProcessResult pending = bootstrap;
      pending.exit_code = 1;
      pending.output =
          "WSL/Ubuntu installation completed, but the default WSL "
          "distribution is not ready yet. Restart Windows if requested, "
          "then click Install Verilator again.";
      pending.error = pending.output;
      return pending;
    }
#else
    return ready;
#endif
  }

  AudioRtlProcessResult existing = Probe(L"/usr/bin/verilator", {L"--version"});
  if (existing.Succeeded()) {
    existing.output = "Verilator is already installed.\n" + existing.output;
    return existing;
  }

  AudioRtlWslCommand update;
  update.run_as_root = true;
  update.program = L"/usr/bin/apt-get";
  update.arguments = {L"update"};
  AudioRtlProcessResult result = Run(update, 10 * 60 * 1000);
  if (!result.Succeeded()) return result;

  AudioRtlWslCommand install = update;
  install.arguments = {L"install", L"-y", L"verilator"};
  AudioRtlProcessResult installed = Run(install, 20 * 60 * 1000);
  installed.output = result.output + "\n" + installed.output;
  if (!installed.Succeeded()) return installed;

  AudioRtlProcessResult verify = Probe(L"/usr/bin/verilator", {L"--version"});
  if (!verify.Succeeded()) {
    installed.exit_code = 1;
    installed.error = "Verilator installation completed, but /usr/bin/verilator is unavailable.";
    installed.output += "\n" + installed.error;
    return installed;
  }
  installed.output += "\nVerilator installation verified.\n" + verify.output;
  return installed;
}

AudioRtlProcessResult AudioRtlWslToolchain::Remove() {
  const AudioRtlProcessResult verilator =
      Probe(L"/usr/bin/verilator", {L"--version"});
  const AudioRtlProcessResult compiler = Probe(L"/usr/bin/g++", {L"--version"});
  const AudioRtlProcessResult make = Probe(L"/usr/bin/make", {L"--version"});

  if (!verilator.Succeeded() && !compiler.Succeeded() && !make.Succeeded()) {
    AudioRtlProcessResult already_removed;
    already_removed.started = true;
    already_removed.exit_code = 0;
    already_removed.output =
        "Verilator, g++, and make are already absent from the default WSL distribution.";
    return already_removed;
  }

  AudioRtlWslCommand remove;
  remove.run_as_root = true;
  remove.program = L"/usr/bin/apt-get";
  remove.arguments = {L"remove", L"--purge", L"-y", L"verilator", L"g++", L"make"};
  AudioRtlProcessResult result = Run(remove, 15 * 60 * 1000);
  if (!result.Succeeded()) return result;

  const AudioRtlProcessResult verify_verilator =
      Probe(L"/usr/bin/verilator", {L"--version"});
  const AudioRtlProcessResult verify_compiler =
      Probe(L"/usr/bin/g++", {L"--version"});
  const AudioRtlProcessResult verify_make =
      Probe(L"/usr/bin/make", {L"--version"});

  std::vector<std::string> remaining;
  if (verify_verilator.Succeeded()) remaining.emplace_back("verilator");
  if (verify_compiler.Succeeded()) remaining.emplace_back("g++");
  if (verify_make.Succeeded()) remaining.emplace_back("make");
  if (!remaining.empty()) {
    result.exit_code = 1;
    std::ostringstream message;
    message << "Some WSL RTL tools are still available after package removal: ";
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      if (i != 0) message << ", ";
      message << remaining[i];
    }
    message << ".";
    result.error = message.str();
    if (!result.output.empty()) result.output += "\n";
    result.output += result.error;
    return result;
  }

  if (!result.output.empty()) result.output += "\n";
  result.output +=
      "Verilator, g++, and make removed. WSL and the Linux distribution were kept.";
  return result;
}

std::string AudioRtlWslToolchain::WindowsPathToWsl(const std::string& path) {
  if (path.size() < 3 || path[1] != ':') return path;
  std::string result = "/mnt/";
  result.push_back(static_cast<char>(
      std::tolower(static_cast<unsigned char>(path.front()))));
  for (std::size_t i = 2; i < path.size(); ++i) {
    result.push_back(path[i] == '\\' ? '/' : path[i]);
  }
  return result;
}

std::string AudioRtlWslToolchain::BuildLaunchCommand(
    const AudioRtlWslCommand& command) {
  std::wstring line = L"wsl.exe";
  for (const auto& argument : BuildArguments(command)) {
    line.push_back(L' ');
    line += QuoteWindowsArgument(argument);
  }
  return WideToUtf8(line);
}

}  // namespace plc
