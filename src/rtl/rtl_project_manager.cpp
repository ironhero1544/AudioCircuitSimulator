/*
 * rtl_project_manager.cpp
 *
 * RTL project library management and lightweight Verilog analysis helpers.
 */

#include "plc_emulator/rtl/rtl_project_manager.h"
#include "plc_emulator/core/windows_power_utils.h"
#include "plc_emulator/rtl/audio_rtl_wsl_toolchain.h"

#include "nlohmann/json.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#include <processenv.h>
#endif

using json = nlohmann::json;

namespace plc {

namespace {
// --- Internal Constants ---

#ifdef _WIN32
static constexpr DWORD kLocalDefaultTimeoutMs = 30000;
static constexpr DWORD kLocalBuildTimeoutMs = 300000;
#else
static constexpr unsigned int kLocalDefaultTimeoutMs = 30000;
static constexpr unsigned int kLocalBuildTimeoutMs = 300000;
#endif
static constexpr size_t kMaxStoredRtlLogBytes = 256 * 1024;

// --- Internal Utilities (Private Linkage) ---

static bool LocalFileExists(const std::string& path) {
  if (path.empty()) return false;
#ifdef _WIN32
  DWORD attrs = GetFileAttributesA(path.c_str());
  return (attrs != INVALID_FILE_ATTRIBUTES && (attrs & FILE_ATTRIBUTE_DIRECTORY) == 0);
#else
  std::ifstream f(path.c_str()); return f.good();
#endif
}

static bool LocalPathExists(const std::string& path) {
  if (path.empty()) return false;
#ifdef _WIN32
  return GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES;
#else
  std::ifstream f(path.c_str()); return f.good();
#endif
}

static std::string LocalJoinPath(const std::string& left, const std::string& right) {
  if (left.empty()) return right;
  std::string res = left;
  if (res.back() != '\\' && res.back() != '/') res += "\\";
  res += right; return res;
}

static std::string LocalToForwardSlashes(const std::string& path) {
  std::string res = path;
  std::replace(res.begin(), res.end(), '\\', '/');
  return res;
}

static std::string LocalTrim(const std::string& text) {
  if (text.empty()) return "";
  size_t start = 0;
  while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) ++start;
  size_t end = text.size();
  while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1]))) --end;
  return text.substr(start, end - start);
}

static void LocalCapStoredLog(std::string* text) {
  if (!text || text->size() <= kMaxStoredRtlLogBytes) return;
  static const std::string kPrefix =
      "[Log truncated. Showing the most recent output.]\n";
  size_t keep = kMaxStoredRtlLogBytes > kPrefix.size()
                    ? kMaxStoredRtlLogBytes - kPrefix.size()
                    : 0;
  size_t start = text->size() > keep ? text->size() - keep : 0;
  *text = kPrefix + text->substr(start);
}

static void LocalCapEntryLogs(plc::RtlLibraryEntry* entry) {
  if (!entry) return;
  LocalCapStoredLog(&entry->diagnostics);
  LocalCapStoredLog(&entry->buildDiagnostics);
  LocalCapStoredLog(&entry->testbenchDiagnostics);
  LocalCapStoredLog(&entry->testbenchBuildLog);
  LocalCapStoredLog(&entry->testbenchRunLog);
}

static json LocalSerializePort(const plc::RtlPortDescriptor& port) {
  return {{"pin_name", port.pinName},
          {"base_name", port.baseName},
          {"bit_index", port.bitIndex},
          {"width", port.width},
          {"is_input", port.isInput},
          {"is_clock", port.isClock},
          {"is_reset", port.isReset},
          {"port_id", port.portId}};
}

static json LocalSerializeEntry(const plc::RtlLibraryEntry& entry,
                                bool includeSourceFiles) {
  json item;
  item["module_id"] = entry.moduleId;
  item["display_name"] = entry.displayName;
  item["top_module"] = entry.topModule;
  item["top_file"] = entry.topFile;
  item["build_hash"] = entry.buildHash;
  item["worker_protocol_version"] = entry.workerProtocolVersion;
  item["analyze_success"] = entry.analyzeSuccess;
  item["build_success"] = entry.buildSuccess;
  item["testbench_top_module"] = entry.testbenchTopModule;
  item["testbench_top_file"] = entry.testbenchTopFile;
  item["diagnostics"] = entry.diagnostics;
  item["build_diagnostics"] = entry.buildDiagnostics;
  item["testbench_diagnostics"] = entry.testbenchDiagnostics;
  item["testbench_build_log"] = entry.testbenchBuildLog;
  item["testbench_run_log"] = entry.testbenchRunLog;
  item["testbench_vcd_path"] = entry.testbenchVcdPath;
  item["testbench_success"] = entry.testbenchSuccess;
  item["component_enabled"] = entry.componentEnabled;

  json sources = json::array();
  if (includeSourceFiles) {
    for (const auto& file : entry.sourceFiles) {
      sources.push_back({{"path", file.path}, {"content", file.content}});
    }
  }
  item["source_files"] = sources;

  json ports = json::array();
  for (const auto& port : entry.ports) {
    ports.push_back(LocalSerializePort(port));
  }
  item["ports"] = std::move(ports);
  return item;
}

static bool LocalDeserializeEntry(const json& item,
                                  plc::RtlLibraryEntry* entry) {
  if (!entry || !item.is_object()) {
    return false;
  }
  const std::string moduleId = item.value("module_id", "");
  if (moduleId.empty()) {
    return false;
  }

  plc::RtlLibraryEntry parsed;
  parsed.moduleId = moduleId;
  parsed.displayName = item.value("display_name", "RTL Module");
  parsed.topModule = item.value("top_module", "");
  parsed.topFile = item.value("top_file", "");
  parsed.buildHash = item.value("build_hash", "");
  parsed.workerProtocolVersion = item.value("worker_protocol_version", 1);
  parsed.analyzeSuccess = item.value("analyze_success", false);
  parsed.buildSuccess = item.value("build_success", false);
  parsed.testbenchTopModule = item.value("testbench_top_module", "");
  parsed.testbenchTopFile = item.value("testbench_top_file", "");
  parsed.diagnostics = item.value("diagnostics", "");
  parsed.buildDiagnostics = item.value("build_diagnostics", "");
  parsed.testbenchDiagnostics = item.value("testbench_diagnostics", "");
  parsed.testbenchBuildLog = item.value("testbench_build_log", "");
  parsed.testbenchRunLog = item.value("testbench_run_log", "");
  parsed.testbenchVcdPath = item.value("testbench_vcd_path", "");
  parsed.testbenchSuccess = item.value("testbench_success", false);
  parsed.componentEnabled = item.value("component_enabled", false);

  auto sourcesIt = item.find("source_files");
  if (sourcesIt != item.end() && sourcesIt->is_array()) {
    for (const auto& source : *sourcesIt) {
      parsed.sourceFiles.push_back(
          {source.value("path", ""), source.value("content", "")});
    }
  }

  auto portsIt = item.find("ports");
  if (portsIt != item.end() && portsIt->is_array()) {
    for (const auto& port : *portsIt) {
      plc::RtlPortDescriptor descriptor;
      descriptor.pinName = port.value("pin_name", "");
      descriptor.baseName = port.value("base_name", "");
      descriptor.bitIndex = port.value("bit_index", -1);
      descriptor.width = port.value("width", 1);
      descriptor.isInput = port.value("is_input", true);
      descriptor.isClock = port.value("is_clock", false);
      descriptor.isReset = port.value("is_reset", false);
      descriptor.portId = port.value("port_id", -1);
      parsed.ports.push_back(std::move(descriptor));
    }
  }

  LocalCapEntryLogs(&parsed);
  *entry = std::move(parsed);
  return true;
}

static std::vector<std::string> LocalSplitPortDeclarations(const std::string& text) {
  std::vector<std::string> declarations;
  std::string current;
  int bracketDepth = 0;
  for (char ch : text) {
    if (ch == '[') ++bracketDepth;
    else if (ch == ']') bracketDepth = std::max(0, bracketDepth - 1);

    if (ch == ',' && bracketDepth == 0) {
      std::string token = LocalTrim(current);
      if (!token.empty()) declarations.push_back(token);
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  std::string token = LocalTrim(current);
  if (!token.empty()) declarations.push_back(token);
  return declarations;
}

static size_t LocalSkipWhitespace(const std::string& text, size_t position) {
  while (position < text.size() &&
         std::isspace(static_cast<unsigned char>(text[position]))) {
    ++position;
  }
  return position;
}

static bool LocalFindMatchingParenthesis(const std::string& text,
                                         size_t openPosition,
                                         size_t* closePosition) {
  if (!closePosition || openPosition >= text.size() ||
      text[openPosition] != '(') {
    return false;
  }

  int depth = 0;
  for (size_t position = openPosition; position < text.size(); ++position) {
    if (text[position] == '(') {
      ++depth;
    } else if (text[position] == ')') {
      --depth;
      if (depth == 0) {
        *closePosition = position;
        return true;
      }
    }
  }
  return false;
}

static bool LocalExtractAnsiModulePorts(const std::string& source,
                                        const std::string& requestedModule,
                                        std::string* detectedModule,
                                        std::string* portsText) {
  if (!detectedModule || !portsText) {
    return false;
  }

  static const std::regex kModuleRegex(
      "\\bmodule\\s+([a-zA-Z_][a-zA-Z0-9_$]*)\\b",
      std::regex::icase);
  for (std::sregex_iterator it(source.begin(), source.end(), kModuleRegex), end;
       it != end; ++it) {
    const std::string moduleName = (*it)[1].str();
    if (!requestedModule.empty() && moduleName != requestedModule) {
      continue;
    }

    size_t cursor = static_cast<size_t>(it->position() + it->length());
    cursor = LocalSkipWhitespace(source, cursor);

    // ANSI parameterized modules place #(parameter ...) between the module
    // name and the actual port list.  Skip the complete balanced parameter
    // block instead of treating its opening parenthesis as the port list.
    if (cursor < source.size() && source[cursor] == '#') {
      cursor = LocalSkipWhitespace(source, cursor + 1);
      size_t parameterClose = 0;
      if (cursor >= source.size() || source[cursor] != '(' ||
          !LocalFindMatchingParenthesis(source, cursor, &parameterClose)) {
        continue;
      }
      cursor = LocalSkipWhitespace(source, parameterClose + 1);
    }

    size_t portClose = 0;
    if (cursor >= source.size() || source[cursor] != '(' ||
        !LocalFindMatchingParenthesis(source, cursor, &portClose)) {
      continue;
    }
    const size_t semicolon = LocalSkipWhitespace(source, portClose + 1);
    if (semicolon >= source.size() || source[semicolon] != ';') {
      continue;
    }

    *detectedModule = moduleName;
    *portsText = source.substr(cursor + 1, portClose - cursor - 1);
    return true;
  }
  return false;
}

static bool LocalEqualsIgnoreCase(const std::string& lhs, const std::string& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
        std::tolower(static_cast<unsigned char>(rhs[i]))) return false;
  }
  return true;
}

static bool LocalSplitDisplayNameSuffix(const std::string& displayName,
                                        std::string* baseName,
                                        int* suffix) {
  const std::string trimmed = LocalTrim(displayName);
  if (baseName) {
    *baseName = trimmed;
  }
  if (suffix) {
    *suffix = 0;
  }

  static const std::regex kSuffixRegex(R"(^(.*)\s\((\d+)\)$)");
  std::smatch match;
  if (!std::regex_match(trimmed, match, kSuffixRegex) || match.size() != 3) {
    return false;
  }

  const std::string parsedBase = LocalTrim(match[1].str());
  if (parsedBase.empty()) {
    return false;
  }

  if (baseName) {
    *baseName = parsedBase;
  }
  if (suffix) {
    *suffix = std::atoi(match[2].str().c_str());
  }
  return true;
}

static bool LocalDisplayNameBelongsToFamily(const std::string& candidate,
                                            const std::string& familyBase,
                                            int* suffix) {
  std::string candidateBase;
  int candidateSuffix = 0;
  const bool hasSuffix =
      LocalSplitDisplayNameSuffix(candidate, &candidateBase, &candidateSuffix);
  if (!LocalEqualsIgnoreCase(candidateBase, familyBase)) {
    return false;
  }

  if (suffix) {
    *suffix = hasSuffix ? candidateSuffix : 0;
  }
  return true;
}

static Color LocalGetRtlPortColor(bool isInput) {
  return isInput ? Color{0.10f, 0.55f, 0.92f, 1.0f}
                 : Color{0.93f, 0.40f, 0.12f, 1.0f};
}

static std::string LocalEscape(const std::string& arg) {
  if (arg.empty()) return "\"\"";
  std::string escaped = "\"";
  for (size_t i = 0; i < arg.size(); ++i) {
    size_t backslashes = 0;
    while (i < arg.size() && arg[i] == '\\') { i++; backslashes++; }
    if (i == arg.size()) {
      for (size_t j = 0; j < backslashes * 2; ++j) escaped.push_back('\\');
    } else if (arg[i] == '"') {
      for (size_t j = 0; j < backslashes * 2 + 1; ++j) escaped.push_back('\\');
      escaped.push_back('"');
    } else {
      for (size_t j = 0; j < backslashes; ++j) escaped.push_back('\\');
      escaped.push_back(arg[i]);
    }
  }
  escaped.push_back('"');
  return escaped;
}

#ifdef _WIN32
static std::wstring LocalToWide(const std::string& s) {
  if (s.empty()) return L"";
  int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
  if (n <= 0) return L"";
  std::wstring res(n, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &res[0], n);
  if (!res.empty() && res.back() == L'\0') res.pop_back();
  return res;
}

static DWORD GetPowerAwareTimeout(DWORD timeout) {
  return plc::GetPowerAwareTimeout(timeout);
}
#endif

static bool LocalRunCommand(const std::string& cmd, std::string* out, DWORD timeout) {
  if (out) out->clear();
  if (cmd.empty()) return false;
#ifdef _WIN32
  timeout = GetPowerAwareTimeout(timeout);
  SECURITY_ATTRIBUTES sa; memset(&sa, 0, sizeof(sa)); sa.nLength = sizeof(sa); sa.bInheritHandle = TRUE;
  HANDLE rPipe = nullptr, wPipe = nullptr;
  if (!CreatePipe(&rPipe, &wPipe, &sa, 0)) return false;
  SetHandleInformation(rPipe, HANDLE_FLAG_INHERIT, 0);
  HANDLE nIn = CreateFileA("NUL", GENERIC_READ, FILE_SHARE_READ, &sa, OPEN_EXISTING, 0, nullptr);
  STARTUPINFOW si; memset(&si, 0, sizeof(si)); si.cb = sizeof(si);
  si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES; si.wShowWindow = SW_HIDE;
  si.hStdInput = (nIn != INVALID_HANDLE_VALUE ? nIn : nullptr); si.hStdOutput = si.hStdError = wPipe;
  PROCESS_INFORMATION pi; memset(&pi, 0, sizeof(pi));
  std::wstring wcmd = LocalToWide(cmd);
  if (!CreateProcessW(nullptr, &wcmd[0], nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
    CloseHandle(rPipe); CloseHandle(wPipe); if (nIn != INVALID_HANDLE_VALUE) CloseHandle(nIn);
    if (out) {
      DWORD err = GetLastError();
      char msg[512] = {0};
      FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                     nullptr, err, 0, msg, sizeof(msg), nullptr);
      *out = std::string("CreateProcess failed (") + std::to_string(err) + "): " + msg + "\nCommand: " + cmd;
    }
    return false;
  }
  ApplyWindowsEfficiencyModeCompatibility(pi.hProcess, pi.hThread);
  CloseHandle(wPipe); if (nIn != INVALID_HANDLE_VALUE) CloseHandle(nIn);
  std::string captured; DWORD start = GetTickCount(); bool tO = false;
  while (true) {
    DWORD avail = 0;
    if (PeekNamedPipe(rPipe, nullptr, 0, nullptr, &avail, nullptr) && avail > 0) {
      std::vector<char> buf(avail); DWORD read = 0;
      if (ReadFile(rPipe, buf.data(), avail, &read, nullptr)) captured.append(buf.data(), read);
    }
    if (WaitForSingleObject(pi.hProcess, 50) != WAIT_TIMEOUT) break;
    if (GetTickCount() - start > timeout) { tO = true; TerminateProcess(pi.hProcess, 1); break; }
  }
  DWORD fDrainStart = GetTickCount();
  while (GetTickCount() - fDrainStart < 500) {
    DWORD avail = 0;
    if (PeekNamedPipe(rPipe, nullptr, 0, nullptr, &avail, nullptr) && avail > 0) {
      std::vector<char> buf(avail); DWORD read = 0;
      if (ReadFile(rPipe, buf.data(), avail, &read, nullptr)) captured.append(buf.data(), read);
    } else break;
  }
  DWORD exit = 1; GetExitCodeProcess(pi.hProcess, &exit);
  CloseHandle(pi.hThread); CloseHandle(pi.hProcess); CloseHandle(rPipe);

#ifdef _WIN32
  {
    int wlen = MultiByteToWideChar(CP_OEMCP, 0, captured.c_str(), static_cast<int>(captured.size()), nullptr, 0);
    if (wlen > 0) {
      std::wstring wbuf(wlen, L'\0');
      MultiByteToWideChar(CP_OEMCP, 0, captured.c_str(), static_cast<int>(captured.size()), &wbuf[0], wlen);
      int ulen = WideCharToMultiByte(CP_UTF8, 0, wbuf.c_str(), wlen, nullptr, 0, nullptr, nullptr);
      if (ulen > 0) {
        std::string utf8(ulen, '\0');
        WideCharToMultiByte(CP_UTF8, 0, wbuf.c_str(), wlen, &utf8[0], ulen, nullptr, nullptr);
        captured = std::move(utf8);
      }
    }
  }
#endif

  if (out) {
    *out = LocalTrim(captured);
    if (tO) *out += "\nCommand timed out.";
    if (exit != 0 && out->empty()) {
      char hexBuf[32];
      snprintf(hexBuf, sizeof(hexBuf), "%lX", static_cast<unsigned long>(exit));
      *out = "Process crashed or exited silently with code " + std::to_string(exit) + " (0x" + hexBuf + ").\nIf the code is 0xC0000135, a required DLL (e.g., libintl-8.dll) is missing from the toolchain.";
    }
  }
  return !tO && exit == 0;
#else
  (void)timeout; return false;
#endif
}

static std::string LocalHashHex(const std::string& text) {
  uint64_t h = 1469598103934665603ULL;
  for (unsigned char ch : text) { h ^= static_cast<uint64_t>(ch); h *= 1099511628211ULL; }
  std::ostringstream ss; ss << std::hex << std::setfill('0') << std::setw(16) << h;
  return ss.str();
}

static std::string LocalStripComments(const std::string& text) {
  std::string s; bool lC = false, bC = false;
  for (size_t i = 0; i < text.size(); ++i) {
    char ch = text[i], nx = (i + 1 < text.size()) ? text[i + 1] : '\0';
    if (!lC && !bC && ch == '/' && nx == '/') { lC = true; ++i; continue; }
    if (!lC && !bC && ch == '/' && nx == '*') { bC = true; ++i; continue; }
    if (lC) { if (ch == '\n') { lC = false; s += ch; } continue; }
    if (bC) { if (ch == '*' && nx == '/') { bC = false; ++i; } continue; }
    s += ch;
  }
  return s;
}

static std::string LocalWinPathToWslPath(const std::string& path) {
  if (path.size() < 3 || path[1] != ':') return path;
  std::string res = "/mnt/";
  res.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(path[0]))));
  for (size_t i = 2; i < path.size(); ++i) res.push_back(path[i] == '\\' ? '/' : path[i]);
  return res;
}

static bool LocalWriteFileContent(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary); if (!f.is_open()) return false;
  f << content; return true;
}

static std::string LocalGetTempBasePath(const std::string& suffix) {
#ifdef _WIN32
  char buf[MAX_PATH] = {0}; DWORD len = GetTempPathA(MAX_PATH, buf);
  if (len > 0 && len < MAX_PATH) {
    std::string p(buf, len); if (!p.empty() && p.back() != '\\' && p.back() != '/') p += "\\";
    p += suffix; return p;
  }
#endif
  return suffix;
}

static std::string LocalGetDirectoryName(const std::string& path) {
  size_t pos = path.find_last_of("\\/");
  if (pos == std::string::npos) return "";
  return path.substr(0, pos);
}

static bool LocalEnsureDirRecursive(const std::string& path) {
  if (path.empty()) return false;
  std::string norm = path; std::replace(norm.begin(), norm.end(), '/', '\\');
#ifdef _WIN32
  size_t pos = norm.find('\\', 3);
  while (pos != std::string::npos) { CreateDirectoryA(norm.substr(0, pos).c_str(), nullptr); pos = norm.find('\\', pos + 1); }
  if (CreateDirectoryA(norm.c_str(), nullptr) != 0) return true;
  return GetLastError() == ERROR_ALREADY_EXISTS;
#else
  return true;
#endif
}

static std::string GetLocalAppDataPathInternal() {
#ifdef _WIN32
  char buf[MAX_PATH] = {0}; if (GetEnvironmentVariableA("LOCALAPPDATA", buf, MAX_PATH) > 0) return std::string(buf);
#endif
  return ".";
}

static std::string BuildCacheRootDirectory() {
  return LocalJoinPath(GetLocalAppDataPathInternal(),
                       "AudioCircuitSimulator\\rtl_cache");
}

static std::string LocalReadFileContent(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) return "";
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static const std::vector<std::string>& GetRtlWorkerRuntimeFileNames() {
  static const std::vector<std::string> kNames = {
      "libgcc_s_seh-1.dll",
      "libstdc++-6.dll",
      "libwinpthread-1.dll",
  };
  return kNames;
}

static bool LocalIsDedicatedAudioShell(const std::string& module_id) {
  return module_id == "audio_shell_adc" ||
         module_id == "audio_shell_dac" ||
         module_id == "audio_shell_amplifier";
}

} // anonymous namespace

// --- Public Function ---
bool FileExists(const std::string& path) { return LocalFileExists(path); }

// --- RtlProjectManager Implementation ---

RtlProjectManager::RtlProjectManager() {
  LoadGlobalLibrary();
  ResetProjectAudioComponentShells();
}
RtlProjectManager::~RtlProjectManager() { SaveGlobalLibrary(); }

void RtlProjectManager::LoadGlobalLibrary() {
  std::ifstream f(LocalJoinPath(GetLocalAppDataPathInternal(),
                                "AudioCircuitSimulator\\rtl_library.json"),
                  std::ios::binary);
  if (!f.is_open()) return;
  std::ostringstream ss; ss << f.rdbuf();
  if (!ss.str().empty()) DeserializeLibraryJson(ss.str());
}

void RtlProjectManager::SaveGlobalLibrary() const {
  std::string p = LocalJoinPath(GetLocalAppDataPathInternal(),
                                "AudioCircuitSimulator\\rtl_library.json");
  size_t pos = p.find_last_of("\\/");
  if (pos != std::string::npos) LocalEnsureDirRecursive(p.substr(0, pos));
  // ADC/DAC/OP-AMP sources are project assets. Persisting them here leaked a
  // previously opened project's Verilog into a fresh, empty session.
  json root;
  root["version"] = 1;
  json entries = json::array();
  for (const RtlLibraryEntry& entry : library_) {
    if (!LocalIsDedicatedAudioShell(entry.moduleId)) {
      entries.push_back(LocalSerializeEntry(entry, true));
    }
  }
  root["entries"] = std::move(entries);
  std::ofstream f(p, std::ios::binary);
  if (f.is_open()) {
    f << root.dump(2, ' ', false, json::error_handler_t::replace);
  }
}

void RtlProjectManager::ResetProjectAudioComponentShells() {
  const auto first_dedicated = std::remove_if(
      library_.begin(), library_.end(), [](const RtlLibraryEntry& entry) {
        return LocalIsDedicatedAudioShell(entry.moduleId);
      });
  if (first_dedicated != library_.end()) {
    library_.erase(first_dedicated, library_.end());
    InvalidateLibraryIndex();
  }
  EnsureAudioComponentShells();
}

void RtlProjectManager::InvalidateLibraryIndex() {
  libraryIndexById_.clear();
  libraryIndexValid_ = false;
}

void RtlProjectManager::RebuildLibraryIndex() const {
  if (libraryIndexValid_) {
    return;
  }
  libraryIndexById_.clear();
  libraryIndexById_.reserve(library_.size());
  for (size_t i = 0; i < library_.size(); ++i) {
    libraryIndexById_[library_[i].moduleId] = i;
  }
  libraryIndexValid_ = true;
}

RtlLibraryEntry* RtlProjectManager::FindEntryById(const std::string& id) {
  RebuildLibraryIndex();
  const auto it = libraryIndexById_.find(id);
  if (it == libraryIndexById_.end() || it->second >= library_.size()) {
    return nullptr;
  }
  return &library_[it->second];
}

const RtlLibraryEntry* RtlProjectManager::FindEntryById(const std::string& id) const {
  RebuildLibraryIndex();
  const auto it = libraryIndexById_.find(id);
  if (it == libraryIndexById_.end() || it->second >= library_.size()) {
    return nullptr;
  }
  return &library_[it->second];
}

RtlLibraryEntry* RtlProjectManager::CreateEntry(const std::string& name) {
  RtlLibraryEntry e;
  e.displayName = BuildUniqueDisplayName(name.empty() ? "RTL Module" : name);
  e.moduleId = BuildModuleId(e.displayName, library_.size());
  e.topModule = Slugify(e.displayName); if (e.topModule.empty()) e.topModule = "rtl_top";
  e.topFile = e.topModule + ".v";
  e.sourceFiles.push_back(
      {e.topFile,
       "module " + e.topModule +
           "();\n\n// Add input/output ports, then run Analyze and Build.\n\nendmodule\n"});
  e.componentEnabled = true;
  library_.push_back(e);
  InvalidateLibraryIndex();
  SaveGlobalLibrary();
  return &library_.back();
}

void RtlProjectManager::EnsureAudioComponentShells() {
  struct ShellDefinition {
    const char* id;
    const char* name;
    const char* top;
  };
  static constexpr ShellDefinition kShells[] = {
      {"audio_shell_adc", "ADC", "audio_adc"},
      {"audio_shell_dac", "DAC", "audio_dac"},
      {"audio_shell_amplifier", "OP-AMP", "audio_amplifier"},
  };

  bool changed = false;
  const auto deprecated_begin = std::remove_if(
      library_.begin(), library_.end(), [](const RtlLibraryEntry& entry) {
        return entry.moduleId == "audio_shell_volume" ||
               entry.moduleId == "audio_shell_filter";
      });
  if (deprecated_begin != library_.end()) {
    library_.erase(deprecated_begin, library_.end());
    InvalidateLibraryIndex();
    changed = true;
  }
  for (const ShellDefinition& shell : kShells) {
    if (RtlLibraryEntry* existing = FindEntryById(shell.id)) {
      if (existing->displayName != shell.name) {
        existing->displayName = shell.name;
        changed = true;
      }
      continue;
    }
    RtlLibraryEntry entry;
    entry.moduleId = shell.id;
    entry.displayName = shell.name;
    entry.topModule = shell.top;
    entry.topFile = entry.topModule + ".v";
    entry.componentEnabled = true;
    entry.sourceFiles.push_back(
        {entry.topFile,
         "module " + entry.topModule +
             "();\n\n// This component starts with zero pins.\n"
             "// Declare ports here, then run Analyze and Build.\n\nendmodule\n"});
    library_.push_back(std::move(entry));
    InvalidateLibraryIndex();
    changed = true;
  }
  if (changed) {
    SaveGlobalLibrary();
  }
}

RtlLibraryEntry* RtlProjectManager::ImportEntry(const RtlLibraryEntry& entry) {
  RtlLibraryEntry imported = entry;
  imported.displayName = BuildUniqueDisplayName(imported.displayName);
  if (imported.topModule.empty()) {
    imported.topModule = Slugify(imported.displayName);
    if (imported.topModule.empty()) {
      imported.topModule = "rtl_top";
    }
  }
  if (imported.moduleId.empty() || FindEntryById(imported.moduleId)) {
    size_t ordinal = library_.size();
    do {
      imported.moduleId = BuildModuleId(imported.displayName, ordinal++);
    } while (FindEntryById(imported.moduleId));
  }
  LocalCapEntryLogs(&imported);
  library_.push_back(std::move(imported));
  InvalidateLibraryIndex();
  SaveGlobalLibrary();
  return &library_.back();
}

bool RtlProjectManager::RenameEntryDisplayName(const std::string& moduleId,
                                               const std::string& displayName) {
  RtlLibraryEntry* entry = FindEntryById(moduleId);
  if (!entry) {
    return false;
  }

  const std::string uniqueDisplayName =
      BuildUniqueDisplayName(displayName, moduleId);
  if (entry->displayName == uniqueDisplayName) {
    return true;
  }

  entry->displayName = uniqueDisplayName;
  SaveGlobalLibrary();
  return true;
}

bool RtlProjectManager::DeleteEntry(const std::string& id) {
  auto it = std::remove_if(library_.begin(), library_.end(), [&](const RtlLibraryEntry& e) { return e.moduleId == id; });
  if (it == library_.end()) return false;
  library_.erase(it, library_.end());
  InvalidateLibraryIndex();
  SaveGlobalLibrary();
  return true;
}

bool RtlProjectManager::DuplicateEntry(const std::string& id) {
  const auto* e = FindEntryById(id); if (!e) return false;
  RtlLibraryEntry c = *e;
  c.displayName = BuildUniqueDisplayName(c.displayName);
  c.moduleId = BuildModuleId(c.displayName, library_.size());
  c.analyzeSuccess = false;
  c.buildSuccess = false;
  c.componentEnabled = true;
  library_.push_back(c);
  InvalidateLibraryIndex();
  SaveGlobalLibrary();
  return true;
}

std::string RtlProjectManager::ExportSourceFile(const std::string& id, const std::string& path) const {
  const auto* e = FindEntryById(id); if (!e) return "";
  for (const auto& f : e->sourceFiles) { if (f.path == path) return f.content; } return "";
}

bool RtlProjectManager::ImportSourceFile(const std::string& id, const std::string& path, const std::string& content) {
  auto* e = FindEntryById(id); if (!e || path.empty()) return false;
  for (auto& f : e->sourceFiles) {
    if (f.path == path) { if (f.content == content) return true; f.content = content; e->analyzeSuccess = e->buildSuccess = false; SaveGlobalLibrary(); return true; }
  }
  e->sourceFiles.push_back({path, content}); e->analyzeSuccess = e->buildSuccess = false; SaveGlobalLibrary(); return true;
}

bool RtlProjectManager::RemoveSourceFile(const std::string& id, const std::string& path) {
  auto* e = FindEntryById(id); if (!e || path.empty() || e->sourceFiles.size() <= 1) return false;
  auto it = std::remove_if(e->sourceFiles.begin(), e->sourceFiles.end(), [&](const RtlSourceFile& f) { return f.path == path; });
  if (it == e->sourceFiles.end()) return false;
  e->sourceFiles.erase(it, e->sourceFiles.end()); if (e->topFile == path) e->topFile = e->sourceFiles[0].path;
  e->analyzeSuccess = e->buildSuccess = false; return true;
}

bool RtlProjectManager::RenameSourceFile(const std::string& id, const std::string& oldP, const std::string& newP) {
  auto* e = FindEntryById(id); if (!e || oldP.empty() || newP.empty() || oldP == newP) return false;
  for (const auto& f : e->sourceFiles) { if (f.path == newP) return false; }
  for (auto& f : e->sourceFiles) {
    if (f.path == oldP) { f.path = newP; if (e->topFile == oldP) e->topFile = newP; e->analyzeSuccess = e->buildSuccess = false; return true; }
  }
  return false;
}

bool RtlProjectManager::UpdateSourceFile(const std::string& id, const std::string& path, const std::string& content) { return ImportSourceFile(id, path, content); }

bool RtlProjectManager::Analyze(const std::string& id) {
  auto* e = FindEntryById(id); if (!e) return false;
  const bool success = AnalyzeDetached(e);
  if (success) SaveGlobalLibrary();
  return success;
}

bool RtlProjectManager::AnalyzeDetached(RtlLibraryEntry* e) const {
  if (!e) return false;
  e->ports.clear(); e->diagnostics.clear(); e->analyzeSuccess = false;
  if (!AnalyzeWithRegex(e)) {
    e->diagnostics = "Port parse failed.";
    LocalCapEntryLogs(e);
    return false;
  }
  auto s = DetectToolchain(); bool ok = true;
  if (s.verilatorFound) {
    std::string d; ok = RunVerilatorLint(*e, s, &d); if (!d.empty()) e->diagnostics = d;
    else if (!ok) e->diagnostics = "Lint failed with no output.";
  } else e->diagnostics = "Lint skipped: " + s.description;
  LocalCapEntryLogs(e);
  e->analyzeSuccess = !e->ports.empty() && ok;
  return e->analyzeSuccess;
}

bool RtlProjectManager::Build(const std::string& id) {
  auto* e = FindEntryById(id); if (!e) return false;
  const bool success = BuildDetached(e);
  if (success) SaveGlobalLibrary();
  return success;
}

bool RtlProjectManager::BuildDetached(RtlLibraryEntry* e) const {
  if (!e) return false;
  e->buildSuccess = false; e->buildDiagnostics.clear();
  auto s = DetectToolchain(); if (!s.verilatorFound || !s.compilerFound) { e->buildDiagnostics = s.description; LocalCapEntryLogs(e); return false; }
  if (!e->analyzeSuccess) { e->buildDiagnostics = "Analyze first."; LocalCapEntryLogs(e); return false; }
  e->buildSuccess = RunVerilatorBuild(e, s, &e->buildDiagnostics);
  LocalCapEntryLogs(e);
  return e->buildSuccess;
}

bool RtlProjectManager::RunTestbench(const std::string& id) {
  auto* e = FindEntryById(id);
  if (!e) return false;
  return RunTestbenchDetached(e);
}

bool RtlProjectManager::RunTestbenchDetached(RtlLibraryEntry* e) const {
  if (!e) return false;

  e->testbenchSuccess = false;
  e->testbenchDiagnostics.clear();
  e->testbenchBuildLog.clear();
  e->testbenchRunLog.clear();
  e->testbenchVcdPath.clear();

  // ✅ TB 파일 지정 여부 먼저 확인
  if (e->testbenchTopFile.empty()) {
    e->testbenchDiagnostics =
        "No testbench file set. Right-click a source file and select 'Set As TB'.";
    LocalCapEntryLogs(e);
    return false;
  }

  // ✅ testbenchTopModule 자동 추출
  if (e->testbenchTopModule.empty()) {
    std::string tbContent;
    for (const auto& f : e->sourceFiles) {
      if (f.path == e->testbenchTopFile) { tbContent = f.content; break; }
    }
    if (!tbContent.empty()) {
      std::string stripped = LocalStripComments(tbContent);  // 버그 수정 후 사용
      std::regex modRe("module\\s+([a-zA-Z_][a-zA-Z0-9_$]*)\\s*[\\(;]",
                       std::regex::icase);
      std::smatch m;
      if (std::regex_search(stripped, m, modRe)) {
        e->testbenchTopModule = m[1].str();
      }
    }
    if (e->testbenchTopModule.empty()) {
      e->testbenchDiagnostics = "Could not determine testbench top module name.";
      LocalCapEntryLogs(e);
      return false;
    }
  }

  auto s = DetectToolchain();
  if (!s.verilatorFound || !s.compilerFound) {
    e->testbenchDiagnostics = s.description;
    LocalCapEntryLogs(e);
    return false;
  }
  e->testbenchSuccess = RunVerilatorTestbench(e, s, &e->testbenchDiagnostics);
  LocalCapEntryLogs(e);
  return e->testbenchSuccess;
}

RtlToolchainStatus RtlProjectManager::DetectToolchain() const {
  RtlToolchainStatus st;
#ifdef _WIN32
  const AudioRtlWslStatus wsl = AudioRtlWslToolchain::Detect();
  st.wslFound = wsl.wsl_found;
  st.wslVerilatorFound = wsl.verilator_found;
  st.wslCompilerFound = wsl.compiler_found;
  st.wslMakeFound = wsl.make_found;
  st.verilatorFound = wsl.verilator_found;
  st.compilerFound = wsl.compiler_found;
  st.makeFound = wsl.make_found;
  st.wslPath = "wsl.exe";
  st.wslVerilatorPath = "/usr/bin/verilator";
  st.wslCompilerPath = "/usr/bin/g++";
  st.wslMakePath = "/usr/bin/make";
  st.verilatorPath = "WSL:/usr/bin/verilator";
  st.compilerPath = "WSL:/usr/bin/g++";
  st.makePath = "WSL:/usr/bin/make";
  st.description = wsl.diagnosis;
#else
  st.description = "Windows only.";
#endif
  return st;
}

bool RtlProjectManager::HasBuildArtifact(const RtlLibraryEntry& e) const {
  if (e.buildHash.empty()) return false;
  return LocalFileExists(LocalJoinPath(GetBuildCacheDirectory(e), "rtl_worker"));
}

std::string RtlProjectManager::GetBuildCacheDirectory(const RtlLibraryEntry& e) const {
  if (e.buildHash.empty()) return "";
  return LocalJoinPath(BuildCacheRootDirectory(), e.buildHash);
}

std::string RtlProjectManager::GetWslBuildCacheDirectory(const RtlLibraryEntry& e) const {
  std::string d = GetBuildCacheDirectory(e); return d.empty() ? "" : LocalWinPathToWslPath(d);
}

std::string RtlProjectManager::GetTestbenchCacheDirectory(const RtlLibraryEntry& e) const {
  if (e.testbenchTopModule.empty()) return "";
  std::ostringstream ss; ss << "tb-v1|" << e.testbenchTopModule << "|" << e.testbenchTopFile;
  for (const auto& f : e.sourceFiles) ss << "|" << f.path << "|" << f.content.size();
  return LocalJoinPath(BuildCacheRootDirectory(), LocalHashHex(ss.str()));
}

std::string RtlProjectManager::GetWslTestbenchCacheDirectory(const RtlLibraryEntry& e) const {
  std::string d = GetTestbenchCacheDirectory(e); return d.empty() ? "" : LocalWinPathToWslPath(d);
}

std::string RtlProjectManager::GetWorkerLaunchCommand(const RtlLibraryEntry& e, const RtlToolchainStatus& s) const {
  (void)s;
  if (!HasBuildArtifact(e)) return "";
  AudioRtlWslCommand command;
  command.working_directory = LocalToWide(GetWslBuildCacheDirectory(e));
  command.program = LocalToWide(
      AudioRtlWslToolchain::WindowsPathToWsl(
          LocalJoinPath(GetBuildCacheDirectory(e), "rtl_worker")));
  return AudioRtlWslToolchain::BuildLaunchCommand(command);
}

std::string RtlProjectManager::SerializeLibraryJson() const {
  json r; r["version"] = 1; json ents = json::array();
  for (const auto& e : library_) {
    ents.push_back(LocalSerializeEntry(e, true));
  }
  r["entries"] = ents; return r.dump(2, ' ', false, json::error_handler_t::replace);
}

std::string RtlProjectManager::SerializeProjectRtlJson() const { return SerializeLibraryJson(); }

std::string RtlProjectManager::SerializeEntryJson(const RtlLibraryEntry& entry,
                                                 bool includeSourceFiles) const {
  return LocalSerializeEntry(entry, includeSourceFiles)
      .dump(2, ' ', false, json::error_handler_t::replace);
}

bool RtlProjectManager::DeserializeLibraryJson(const std::string& jText, bool merge) {
  json r = json::parse(jText, nullptr, false); if (r.is_discarded() || !r.is_object()) return false;
  if (!merge) {
    library_.clear();
    InvalidateLibraryIndex();
  }
  auto it = r.find("entries");
  if (it == r.end() || !it->is_array()) return true;
  for (const auto& i : *it) {
    RtlLibraryEntry e;
    if (!LocalDeserializeEntry(i, &e)) {
      continue;
    }
    auto existingById = std::find_if(
        library_.begin(), library_.end(),
        [&](const RtlLibraryEntry& existing) {
          return existing.moduleId == e.moduleId;
        });
    const bool isDedicatedAudioShell =
        LocalIsDedicatedAudioShell(e.moduleId);
    if (merge && isDedicatedAudioShell && existingById != library_.end()) {
      // Project packages carry the exact analyzed source, port list, and
      // build hash for dedicated audio electronics. Replace only the matching
      // zero-pin shell; custom RTL library entries remain merged as before.
      *existingById = std::move(e);
      continue;
    }
    e.displayName = BuildUniqueDisplayName(e.displayName);
    if (e.moduleId.empty() || std::any_of(library_.begin(), library_.end(),
                                          [&](const RtlLibraryEntry& existing) {
                                            return existing.moduleId == e.moduleId;
                                          })) {
      size_t ordinal = library_.size();
      do {
        e.moduleId = BuildModuleId(e.displayName, ordinal++);
      } while (std::any_of(library_.begin(), library_.end(),
                           [&](const RtlLibraryEntry& existing) {
                             return existing.moduleId == e.moduleId;
                           }));
    }
    library_.push_back(e);
  }
  InvalidateLibraryIndex();
  if (merge) SaveGlobalLibrary();
  return true;
}

std::string RtlProjectManager::BuildUniqueDisplayName(
    const std::string& displayName,
    const std::string& ignoredModuleId) const {
  std::string desiredName = LocalTrim(displayName);
  if (desiredName.empty()) {
    desiredName = "RTL Module";
  }

  const bool hasExactConflict = std::any_of(
      library_.begin(), library_.end(),
      [&](const RtlLibraryEntry& entry) {
        return entry.moduleId != ignoredModuleId &&
               LocalEqualsIgnoreCase(entry.displayName, desiredName);
      });
  if (!hasExactConflict) {
    return desiredName;
  }

  std::string familyBase;
  LocalSplitDisplayNameSuffix(desiredName, &familyBase, nullptr);
  if (familyBase.empty()) {
    familyBase = desiredName;
  }

  std::set<int> usedSuffixes;
  for (const RtlLibraryEntry& entry : library_) {
    if (entry.moduleId == ignoredModuleId) {
      continue;
    }

    int suffix = 0;
    if (!LocalDisplayNameBelongsToFamily(entry.displayName, familyBase,
                                         &suffix)) {
      continue;
    }
    if (suffix > 0) {
      usedSuffixes.insert(suffix);
    }
  }

  int nextSuffix = 1;
  while (usedSuffixes.count(nextSuffix) > 0) {
    ++nextSuffix;
  }
  return familyBase + " (" + std::to_string(nextSuffix) + ")";
}

bool RtlProjectManager::DeserializeEntryJson(const std::string& jsonText,
                                             RtlLibraryEntry* entry) const {
  json item = json::parse(jsonText, nullptr, false);
  if (item.is_discarded() || !item.is_object()) {
    return false;
  }
  return LocalDeserializeEntry(item, entry);
}

std::map<std::string, std::string> RtlProjectManager::GetBuildArtifactBundle(
    const std::string& id) const {
  std::map<std::string, std::string> files;
  const auto* e = FindEntryById(id);
  if (!e || !HasBuildArtifact(*e)) return files;

  const std::string cacheDir = GetBuildCacheDirectory(*e);
  if (cacheDir.empty()) return files;

  const std::string workerPath = LocalJoinPath(cacheDir, "rtl_worker");
  std::string workerData = LocalReadFileContent(workerPath);
  if (workerData.empty()) return files;
  files["rtl_worker"] = std::move(workerData);

  std::string compilerDir;
  RtlToolchainStatus status = DetectToolchain();
  if (!status.compilerPath.empty()) {
    compilerDir = LocalGetDirectoryName(status.compilerPath);
  }

  for (const auto& fileName : GetRtlWorkerRuntimeFileNames()) {
    const std::string cachedPath = LocalJoinPath(cacheDir, fileName);
    std::string data = LocalReadFileContent(cachedPath);
    if (data.empty() && !compilerDir.empty()) {
      data = LocalReadFileContent(LocalJoinPath(compilerDir, fileName));
    }
    if (!data.empty()) {
      files[fileName] = std::move(data);
    }
  }

  return files;
}

bool RtlProjectManager::RestoreBuildArtifactBundle(
    const std::string& id,
    const std::map<std::string, std::string>& files) {
  auto* e = FindEntryById(id);
  if (!e || files.empty()) return false;
  if (e->buildHash.empty()) e->buildHash = LocalHashHex("art-" + id);

  const std::string cacheDir = GetBuildCacheDirectory(*e);
  if (cacheDir.empty() || !LocalEnsureDirRecursive(cacheDir)) return false;

  bool restoredWorker = false;
  for (const auto& [fileName, data] : files) {
    if (fileName.empty() || data.empty()) continue;
    if (fileName.find_first_of("\\/") != std::string::npos) continue;
    const std::string filePath = LocalJoinPath(cacheDir, fileName);
    if (!LocalWriteFileContent(filePath, data)) continue;
    if (LocalEqualsIgnoreCase(fileName, "rtl_worker")) {
      restoredWorker = LocalFileExists(filePath);
    }
  }

  if (restoredWorker) {
    e->buildSuccess = true;
  }
  return restoredWorker;
}

std::string RtlProjectManager::GetBuildArtifactData(const std::string& id) const {
  auto files = GetBuildArtifactBundle(id);
  auto it = files.find("rtl_worker");
  return it == files.end() ? std::string() : it->second;
}

bool RtlProjectManager::RestoreBuildArtifact(const std::string& id, const std::string& data) {
  if (data.empty()) return false;
  return RestoreBuildArtifactBundle(id, {{"rtl_worker", data}});
}

std::vector<Port> RtlProjectManager::BuildRuntimePorts(
    const std::vector<RtlPortDescriptor>& ports) {
  std::vector<Port> runtimePorts;
  runtimePorts.reserve(ports.size());

  int inputCount = 0;
  int outputCount = 0;
  for (const auto& port : ports) {
    if (port.isInput) {
      ++inputCount;
    } else {
      ++outputCount;
    }
  }

  int inputIndex = 0;
  int outputIndex = 0;
  const int maxSideCount = std::max(inputCount, outputCount);
  const float blockWidth = 260.0f;
  const float topMargin = 52.0f;
  const float usableHeight =
      std::max(64.0f, 18.0f * static_cast<float>(std::max(1, maxSideCount - 1)));
  const float inputSpacing =
      inputCount > 1 ? usableHeight / static_cast<float>(inputCount - 1) : 0.0f;
  const float outputSpacing =
      outputCount > 1 ? usableHeight / static_cast<float>(outputCount - 1) : 0.0f;
  for (const auto& port : ports) {
    Port runtimePort;
    runtimePort.id = port.portId;
    runtimePort.color = LocalGetRtlPortColor(port.isInput);
    runtimePort.isInput = port.isInput;
    runtimePort.type = PortType::ELECTRIC;
    runtimePort.role = port.pinName;
    if (port.isInput) {
      float y = topMargin;
      if (inputCount > 1) {
        y += inputSpacing * static_cast<float>(inputIndex);
      } else {
        y += usableHeight * 0.5f;
      }
      runtimePort.relativePos = Position(0.0f, y);
      ++inputIndex;
    } else {
      float y = topMargin;
      if (outputCount > 1) {
        y += outputSpacing * static_cast<float>(outputIndex);
      } else {
        y += usableHeight * 0.5f;
      }
      runtimePort.relativePos = Position(blockWidth, y);
      ++outputIndex;
    }
    runtimePorts.push_back(runtimePort);
  }
  return runtimePorts;
}

std::vector<RtlPinBinding> RtlProjectManager::BuildDefaultPinBindings(
    const std::vector<RtlPortDescriptor>& ports) {
  std::vector<RtlPinBinding> bindings;
  bindings.reserve(ports.size());
  for (const auto& port : ports) {
    bindings.push_back({port.pinName, port.portId, port.isInput});
  }
  return bindings;
}

bool RtlProjectManager::RunCommandCapture(const std::string& cmd, std::string* out, const RtlToolchainStatus& s, unsigned int tO) const {
#ifdef _WIN32
  char oP[32768]; DWORD n = GetEnvironmentVariableA("PATH", oP, 32768);
  std::string prev = (n > 0 && n < 32768) ? std::string(oP, n) : "";
  // MSYS2 usr/bin (perl, python, sh 등)도 함께 포함
  std::string msys2UsrBin = LocalJoinPath(s.msys2Root, "usr\\bin");
  std::string eff = s.mingwBinPath;
  if (!msys2UsrBin.empty() && LocalPathExists(msys2UsrBin)) {
    eff += ";" + msys2UsrBin;
  }
  if (!prev.empty()) eff += ";" + prev;
  SetEnvironmentVariableA("PATH", eff.c_str());
  if (!s.verilatorRoot.empty()) {
    SetEnvironmentVariableA("VERILATOR_ROOT", LocalToForwardSlashes(s.verilatorRoot).c_str());
  }
  if (!s.makePath.empty()) SetEnvironmentVariableA("MAKE", s.makePath.c_str());
  bool ok = LocalRunCommand(cmd, out, static_cast<DWORD>(tO));
  SetEnvironmentVariableA("PATH", prev.empty() ? nullptr : prev.c_str());
  SetEnvironmentVariableA("VERILATOR_ROOT", nullptr); SetEnvironmentVariableA("MAKE", nullptr); return ok;
#else
  return false;
#endif
}

bool RtlProjectManager::AnalyzeWithRegex(RtlLibraryEntry* entry) const {
  if (!entry) {
    return false;
  }
  std::string topContent;
  for (const auto& file : entry->sourceFiles) {
    if (file.path == entry->topFile || topContent.empty()) {
      topContent = file.content;
      if (file.path == entry->topFile) {
        break;
      }
    }
  }
  if (topContent.empty()) {
    return false;
  }
  topContent = LocalStripComments(topContent);
  std::string moduleName;
  std::string portsText;
  if (!LocalExtractAnsiModulePorts(topContent, entry->topModule, &moduleName,
                                   &portsText)) {
    return false;
  }
  entry->topModule = moduleName;
  std::vector<std::string> declarations = LocalSplitPortDeclarations(portsText);
  std::regex declRegex(
      "^\\s*(input|output|inout)\\s+"
      "(?:(?:wire|reg|logic)\\s+)?"
      "(?:\\[\\s*(\\d+)\\s*:\\s*(\\d+)\\s*\\]\\s+)?"
      "([a-zA-Z_][a-zA-Z0-9_$]*)\\s*$",
      std::regex::icase);

  int portId = 0;
  for (const auto& declaration : declarations) {
    std::smatch declMatch;
    if (!std::regex_match(declaration, declMatch, declRegex)) {
      continue;
    }
    std::string direction = declMatch[1].str();
    if (LocalEqualsIgnoreCase(direction, "inout")) {
      entry->diagnostics =
          "inout ports are not supported in v1. Remove or split them into input/output pins.";
      return false;
    }
    bool isInput = LocalEqualsIgnoreCase(direction, "input");
    int left = -1;
    int right = -1;
    if (declMatch[2].matched && declMatch[3].matched) {
      left = std::stoi(declMatch[2].str());
      right = std::stoi(declMatch[3].str());
    }
    std::string name = declMatch[4].str();
    if (left >= 0 && right >= 0) {
      int step = left <= right ? 1 : -1;
      for (int bit = left;; bit += step) {
        RtlPortDescriptor desc;
        desc.baseName = name;
        desc.bitIndex = bit;
        desc.width = std::abs(left - right) + 1;
        desc.pinName = name + "[" + std::to_string(bit) + "]";
        desc.isInput = isInput;
        desc.isClock = (name == "clk" || name == "clock");
        desc.isReset = (name == "rst" || name == "reset" || name == "rst_n");
        desc.portId = portId++;
        entry->ports.push_back(desc);
        if (bit == right) {
          break;
        }
      }
    } else {
      RtlPortDescriptor desc;
      desc.baseName = name;
      desc.pinName = name;
      desc.width = 1;
      desc.isInput = isInput;
      desc.isClock = (name == "clk" || name == "clock");
      desc.isReset = (name == "rst" || name == "reset" || name == "rst_n");
      desc.portId = portId++;
      entry->ports.push_back(desc);
    }
  }

  return !entry->ports.empty();
}

bool RtlProjectManager::RunVerilatorLint(const RtlLibraryEntry& e, const RtlToolchainStatus& s, std::string* diag) const {
  (void)s;
  const std::string temp_dir = LocalGetTempBasePath("audio_rtl_lint\\");
  if (!LocalEnsureDirRecursive(temp_dir)) {
    if (diag) *diag = "Unable to create the RTL lint directory.";
    return false;
  }
  AudioRtlWslCommand command;
  command.working_directory =
      LocalToWide(AudioRtlWslToolchain::WindowsPathToWsl(temp_dir));
  command.program = L"/usr/bin/verilator";
  command.arguments = {L"--lint-only", L"-Wno-fatal", L"-Wno-TIMESCALEMOD",
                       L"--timing", L"--top-module", LocalToWide(e.topModule)};
  for (const auto& source : e.sourceFiles) {
    const std::string destination = LocalJoinPath(temp_dir, source.path);
    const size_t slash = destination.find_last_of("\\/");
    if (slash != std::string::npos) {
      LocalEnsureDirRecursive(destination.substr(0, slash));
    }
    LocalWriteFileContent(destination, source.content);
    command.arguments.push_back(LocalToWide(LocalToForwardSlashes(source.path)));
  }
  const AudioRtlProcessResult result =
      AudioRtlWslToolchain::Run(command, kLocalDefaultTimeoutMs);
  if (diag) {
    *diag = !result.output.empty() ? result.output : result.error;
  }
  return result.Succeeded();
}

bool RtlProjectManager::RunVerilatorBuild(RtlLibraryEntry* e, const RtlToolchainStatus& s, std::string* diag) const {
#ifdef _WIN32
  (void)s;
  {
    std::ostringstream hashSrc;
    hashSrc << "worker-protocol-2|" << e->moduleId << "|" << e->topModule;
    for (const auto& f : e->sourceFiles) {
      hashSrc << "|" << f.path << "|" << f.content;
    }
    e->buildHash = LocalHashHex(hashSrc.str());
    e->workerProtocolVersion = 2;
  }

  std::string d = GetBuildCacheDirectory(*e);
  if (d.empty()) {
    if (diag) *diag = "[Build] Cache path generation failed: buildHash is empty.";
    return false;
  }

  std::string sD = LocalJoinPath(d, "src"), oD = LocalJoinPath(d, "obj");
  LocalEnsureDirRecursive(d); LocalEnsureDirRecursive(sD); LocalEnsureDirRecursive(oD);
  for (const auto& source : e->sourceFiles) {
    const std::string destination = LocalJoinPath(sD, source.path);
    const size_t slash = destination.find_last_of("\\/");
    if (slash != std::string::npos) {
      LocalEnsureDirRecursive(destination.substr(0, slash));
    }
    LocalWriteFileContent(destination, source.content);
  }
  std::string wm = LocalJoinPath(d, "worker.cpp");
  const auto has_port = [&](const char* name, bool is_input) {
    return std::any_of(e->ports.begin(), e->ports.end(),
                       [&](const RtlPortDescriptor& port) {
                         return (port.pinName == name || port.baseName == name) &&
                                port.isInput == is_input;
                       });
  };
  const bool supports_audio_blocks =
      e->moduleId == "audio_shell_dac" && has_port("bclk", true) &&
      has_port("lrclk", true) && has_port("sdata", true) &&
      has_port("rst_n", true) && has_port("dac_l", false) &&
      has_port("dac_r", false);
  std::ostringstream code;
  code << "#include \"V" << e->topModule << ".h\"\n"
       << "#include <verilated.h>\n"
       << "#include <iostream>\n#include <sstream>\n#include <string>\n#include <cstdint>\n#include <vector>\n\n"
       << "static vluint64_t g_main_time = 0;\n"
       << "double sc_time_stamp() { return static_cast<double>(g_main_time); }\n\n"
       << "int main(int argc, char** argv) {\n"
       << "  Verilated::commandArgs(argc, argv);\n"
       << "  std::ios::sync_with_stdio(false); std::cin.tie(nullptr);\n"
       << "  V" << e->topModule << " top;\n";
  if (supports_audio_blocks) {
    code << "  auto eval = [&]() { top.eval(); ++g_main_time; };\n"
         << "  auto clock_bit = [&](int lr, int data) { top.lrclk=lr; top.sdata=data; top.bclk=0; eval(); top.bclk=1; eval(); };\n"
         << "  auto shift_slot = [&](int lr, std::int16_t sample) { std::uint16_t word=static_cast<std::uint16_t>(sample); clock_bit(lr,0); for(int bit=15; bit>=0; --bit) clock_bit(lr,(word>>bit)&1U); for(int padding=0; padding<15; ++padding) clock_bit(lr,0); };\n"
         << "  top.rst_n=0; top.lrclk=1; top.bclk=0; eval(); top.rst_n=1; eval();\n"
         << "  if(argc>1 && std::string(argv[1])==\"--audio-binary\") {\n"
         << "    struct Header { std::uint32_t magic; std::uint16_t version; std::uint16_t type; std::uint32_t sequence; std::uint32_t frames; };\n"
         << "    while(true) { Header request{}; if(!std::cin.read(reinterpret_cast<char*>(&request),sizeof(request))) break; if(request.magic!=0x32524341U || request.version!=2U || request.type!=1U || request.frames>4096U) return 2; std::vector<std::int16_t> pcm(static_cast<std::size_t>(request.frames)*2U); if(!pcm.empty() && !std::cin.read(reinterpret_cast<char*>(pcm.data()),static_cast<std::streamsize>(pcm.size()*sizeof(std::int16_t)))) return 3; std::vector<std::uint16_t> codes(pcm.size()); for(std::uint32_t frame=0;frame<request.frames;++frame){ shift_slot(0,pcm[static_cast<std::size_t>(frame)*2U]); shift_slot(1,pcm[static_cast<std::size_t>(frame)*2U+1U]); codes[static_cast<std::size_t>(frame)*2U]=static_cast<std::uint16_t>(top.dac_l); codes[static_cast<std::size_t>(frame)*2U+1U]=static_cast<std::uint16_t>(top.dac_r); } Header response{0x32534341U,2U,2U,request.sequence,request.frames}; std::cout.write(reinterpret_cast<const char*>(&response),sizeof(response)); if(!codes.empty()) std::cout.write(reinterpret_cast<const char*>(codes.data()),static_cast<std::streamsize>(codes.size()*sizeof(std::uint16_t))); std::cout.flush(); }\n"
         << "    return 0;\n"
         << "  }\n";
  }
  code << "  std::string line;\n"
       << "  while (std::getline(std::cin, line)) {\n"
       << "    if (line == \"QUIT\") break;\n";
  if (supports_audio_blocks) {
    code << "    if (line.rfind(\"AUDIO\",0)==0) { std::istringstream audio(line.substr(5)); std::size_t count=0; audio>>count; std::cout<<\"AUDIO_OUT \"<<count; for(std::size_t frame=0;frame<count;++frame){ int left=0,right=0; audio>>left>>right; shift_slot(0,static_cast<std::int16_t>(left)); shift_slot(1,static_cast<std::int16_t>(right)); const auto code_l=static_cast<std::uint16_t>(top.dac_l); const auto code_r=static_cast<std::uint16_t>(top.dac_r); std::cout<<' '<<((static_cast<float>(code_l)-32768.0f)/32768.0f)<<' '<<((static_cast<float>(code_r)-32768.0f)/32768.0f); } std::cout<<'\\n'<<std::flush; continue; }\n";
  }
  code << "    if (line.rfind(\"EVAL\", 0) == 0) {\n"
       << "      std::istringstream ss(line.substr(4));\n"
       << "      std::string tok;\n"
       << "      while (ss >> tok) {\n"
       << "        auto eq = tok.find('=');\n"
       << "        if (eq == std::string::npos) continue;\n"
       << "        std::string pin = tok.substr(0, eq);\n"
       << "        int val = std::stoi(tok.substr(eq + 1));\n";

  for (const auto& port : e->ports) {
    if (port.isInput) {
      if (port.bitIndex < 0) {
        code << "        if (pin == \"" << port.pinName << "\") top."
             << port.baseName << " = val;\n";
      } else {
        const auto lastBit = std::find_if(
            e->ports.rbegin(), e->ports.rend(),
            [&](const RtlPortDescriptor& candidate) {
              return candidate.baseName == port.baseName &&
                     candidate.isInput == port.isInput &&
                     candidate.width == port.width &&
                     candidate.bitIndex >= 0;
            });
        const int packedOffset =
            lastBit == e->ports.rend()
                ? port.bitIndex
                : std::abs(port.bitIndex - lastBit->bitIndex);
        if (port.width <= 64) {
          code << "        if (pin == \"" << port.pinName << "\") top."
               << port.baseName << " = static_cast<decltype(top."
               << port.baseName << ")>((static_cast<std::uint64_t>(top."
               << port.baseName << ") & ~(std::uint64_t{1} << "
               << packedOffset
               << ")) | ((val != 0 ? std::uint64_t{1} : std::uint64_t{0}) << "
               << packedOffset << "));\n";
        } else {
          code << "        if (pin == \"" << port.pinName << "\") top."
               << port.baseName << "[" << (packedOffset / 32)
               << "] = (top." << port.baseName << "[" << (packedOffset / 32)
               << "] & ~(std::uint32_t{1} << " << (packedOffset % 32)
               << ")) | ((val != 0 ? std::uint32_t{1} : std::uint32_t{0}) << "
               << (packedOffset % 32) << ");\n";
        }
      }
    }
  }

  code << "      }\n";
  if (supports_audio_blocks) {
    code << "      eval();\n";
  } else {
    code << "      top.eval();\n"
         << "      ++g_main_time;\n";
  }
  code
       << "      std::cout << \"OUT\";\n";

  for (const auto& port : e->ports) {
    if (!port.isInput) {
      if (port.bitIndex < 0) {
        code << "      std::cout << \" \" << \"" << port.pinName
             << "=\" << (int)top." << port.baseName << ";\n";
      } else {
        const auto lastBit = std::find_if(
            e->ports.rbegin(), e->ports.rend(),
            [&](const RtlPortDescriptor& candidate) {
              return candidate.baseName == port.baseName &&
                     candidate.isInput == port.isInput &&
                     candidate.width == port.width &&
                     candidate.bitIndex >= 0;
            });
        const int packedOffset =
            lastBit == e->ports.rend()
                ? port.bitIndex
                : std::abs(port.bitIndex - lastBit->bitIndex);
        if (port.width <= 64) {
          code << "      std::cout << \" \" << \"" << port.pinName
               << "=\" << ((static_cast<std::uint64_t>(top."
               << port.baseName << ") >> " << packedOffset
               << ") & std::uint64_t{1});\n";
        } else {
          code << "      std::cout << \" \" << \"" << port.pinName
               << "=\" << ((top." << port.baseName << "["
               << (packedOffset / 32) << "] >> " << (packedOffset % 32)
               << ") & std::uint32_t{1});\n";
        }
      }
    }
  }

  code << "      std::cout << std::endl;\n"
       << "    }\n"
       << "  }\n"
       << "  top.final(); return 0;\n"
       << "}\n";

  LocalWriteFileContent(wm, code.str());
  AudioRtlWslCommand command;
  command.working_directory = LocalToWide(GetWslBuildCacheDirectory(*e));
  command.program = L"/usr/bin/verilator";
  command.arguments = {L"--cc", L"--exe", L"--build", L"-j", L"0",
                       L"-Wno-fatal", L"-Wno-TIMESCALEMOD", L"--timing",
                       L"--top-module", LocalToWide(e->topModule), L"--Mdir",
                       L"obj", L"-o", L"../rtl_worker"};
  for (const auto& source : e->sourceFiles) {
    command.arguments.push_back(
        LocalToWide("src/" + LocalToForwardSlashes(source.path)));
  }
  command.arguments.push_back(L"worker.cpp");
  const AudioRtlProcessResult result =
      AudioRtlWslToolchain::Run(command, kLocalBuildTimeoutMs);
  if (diag) {
    *diag = !result.output.empty() ? result.output : result.error;
  }
  return result.Succeeded() &&
         LocalFileExists(LocalJoinPath(d, "rtl_worker"));
#else
  (void)e; (void)s; (void)diag;
#endif
  return false;
}

bool RtlProjectManager::RunVerilatorTestbench(RtlLibraryEntry* e, const RtlToolchainStatus& s, std::string* diag) const {
#ifdef _WIN32
  // ── 1. 캐시 디렉토리 준비 ──────────────────────────────────────
  std::string d  = GetTestbenchCacheDirectory(*e);
  if (d.empty()) {
    if (diag) *diag = "Failed to get testbench cache directory.";
    return false;
  }
  std::string sD = LocalJoinPath(d, "src");
  std::string oD = LocalJoinPath(d, "obj");
  LocalEnsureDirRecursive(d);
  LocalEnsureDirRecursive(sD);
  LocalEnsureDirRecursive(oD);

  // ── 2. 소스 파일 전체 기록 ────────────────────────────────────
  for (const auto& f : e->sourceFiles) {
    std::string dest = LocalJoinPath(sD, f.path);
    // 하위 디렉토리 포함 경로 처리
    size_t slash = dest.find_last_of("\\/");
    if (slash != std::string::npos) LocalEnsureDirRecursive(dest.substr(0, slash));
    LocalWriteFileContent(dest, f.content);
  }

  // ── 3. C++ 시뮬레이션 드라이버 생성 ─────────────────────────
  std::string vcdAbsPath = LocalToForwardSlashes(LocalJoinPath(d, "dump.vcd"));
  std::string tbMain     = LocalJoinPath(d, "tb_main.cpp");
  std::string tbExe      = LocalJoinPath(d, "tb_sim.exe");

  std::ostringstream code;
  code << "#include \"V" << e->testbenchTopModule << ".h\"\n"
       << "#include <verilated.h>\n"
       << "#include <verilated_vcd_c.h>\n"
       << "#include <cstdint>\n\n"
       << "static uint64_t g_sim_time = 0;\n"
       << "double sc_time_stamp() { return (double)g_sim_time; }\n\n"
       << "int main(int argc, char** argv) {\n"
       << "  Verilated::commandArgs(argc, argv);\n"
       << "  V" << e->testbenchTopModule << " top;\n"
       << "  Verilated::traceEverOn(true);\n"
       << "  VerilatedVcdC vcd;\n"
       << "  top.trace(&vcd, 99);\n"
       << "  vcd.open(\"" << vcdAbsPath << "\");\n"
       // 최대 10M 스텝 (무한루프 방지)
       << "  const uint64_t kMaxSteps = 10000000ULL;\n"
       << "  while (!Verilated::gotFinish() && g_sim_time < kMaxSteps) {\n"
       << "    top.eval();\n"
       << "    vcd.dump(g_sim_time);\n"
       << "    ++g_sim_time;\n"
       << "  }\n"
       << "  top.final();\n"
       << "  vcd.flush(); vcd.close();\n"
       << "  return Verilated::gotFinish() ? 0 : 2;\n"
       << "}\n";
  LocalWriteFileContent(tbMain, code.str());

  // ── 4. Verilator: C++ 코드 생성 ───────────────────────────────
  std::ostringstream vC;
  vC << LocalEscape(s.verilatorPath)
     << " --cc --exe --trace -Wno-fatal -Wno-TIMESCALEMOD --timing"
     << " -I" << LocalEscape(LocalToForwardSlashes(LocalJoinPath(s.verilatorRoot, "include")))
     << " --top-module " << LocalEscape(e->testbenchTopModule)
     << " -Mdir " << LocalEscape(LocalToForwardSlashes(oD));
  for (const auto& f : e->sourceFiles) {
    vC << " " << LocalEscape(LocalToForwardSlashes(LocalJoinPath(sD, f.path)));
  }
  vC << " " << LocalEscape(LocalToForwardSlashes(tbMain));

  std::string buildLog;
  if (!RunCommandCapture(vC.str(), &buildLog, s, kLocalBuildTimeoutMs)) {
    if (diag) *diag = "[Verilator step]\n" + (buildLog.empty()
        ? "Process failed.\nCommand: " + vC.str() : buildLog);
    e->testbenchBuildLog = buildLog;
    return false;
  }
  e->testbenchBuildLog = buildLog;

  // ── 5. g++: 수동 컴파일 ────────────────────────────────────────
  std::string vIncl = LocalToForwardSlashes(LocalJoinPath(s.verilatorRoot, "include"));
  std::ostringstream cC;
  cC << LocalEscape(LocalToForwardSlashes(s.compilerPath))
     << " -std=c++20 -fcoroutines -Os"
     << " -I" << LocalEscape(vIncl)
     << " -I" << LocalEscape(vIncl + "/vltstd")
     << " -I" << LocalEscape(LocalToForwardSlashes(oD))
     << " " << LocalEscape(LocalToForwardSlashes(tbMain))
     << " " << LocalEscape(vIncl + "/verilated.cpp")
     << " " << LocalEscape(vIncl + "/verilated_vcd_c.cpp");

  if (LocalFileExists(LocalToForwardSlashes(vIncl + "/verilated_threads.cpp")))
    cC << " " << LocalEscape(vIncl + "/verilated_threads.cpp");
  if (LocalFileExists(LocalToForwardSlashes(vIncl + "/verilated_timing.cpp")))
    cC << " " << LocalEscape(vIncl + "/verilated_timing.cpp");

  // V{top}*.cpp 파일 추가
  WIN32_FIND_DATAA fd;
  std::string searchPat = oD + "\\V" + e->testbenchTopModule + "*.cpp";
  HANDLE hFind = FindFirstFileA(searchPat.c_str(), &fd);
  if (hFind != INVALID_HANDLE_VALUE) {
    do {
      if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        cC << " " << LocalEscape(LocalToForwardSlashes(LocalJoinPath(oD, fd.cFileName)));
    } while (FindNextFileA(hFind, &fd));
    FindClose(hFind);
  }
  cC << " -o " << LocalEscape(LocalToForwardSlashes(tbExe));

  std::string compileLog;
  if (!RunCommandCapture(cC.str(), &compileLog, s, kLocalBuildTimeoutMs)) {
    e->testbenchBuildLog += "\n[C++ Compile]\n" + (compileLog.empty()
        ? "Compile failed.\nCommand: " + cC.str() : compileLog);
    if (diag) *diag = e->testbenchBuildLog;
    return false;
  }
  e->testbenchBuildLog += "\n[C++ Compile]\n" + compileLog;

  if (!LocalFileExists(tbExe)) {
    if (diag) *diag = "Testbench executable not produced.";
    return false;
  }

  // ── 6. 테스트벤치 실행 ────────────────────────────────────────
  std::string runLog;
  bool ran = RunCommandCapture(LocalEscape(LocalToForwardSlashes(tbExe)),
                               &runLog, s, kLocalDefaultTimeoutMs);
  e->testbenchRunLog = runLog;

  // ── 7. 합격/실패 판정 ─────────────────────────────────────────
  auto HasKeyword = [&](const std::string& kw) {
    return runLog.find(kw) != std::string::npos;
  };
  bool hasFatal = HasKeyword("$fatal") || HasKeyword("%Fatal") ||
                  HasKeyword("Aborting");
  bool hasError = HasKeyword("$error") || HasKeyword("%Error") ||
                  HasKeyword("Assertion failed") ||
                  HasKeyword("assertion failed");

  if (hasFatal || hasError) {
    if (diag) *diag = "Testbench reported errors. Check Testbench Run Log.";
    return false;
  }
  if (!ran) {
    if (diag) *diag = "Testbench timed out or crashed. Check Testbench Run Log.";
    return false;
  }

  // VCD 파일 기록
  if (LocalFileExists(vcdAbsPath)) e->testbenchVcdPath = vcdAbsPath;

  if (diag) *diag = "Testbench passed.";
  return true;
#else
  (void)e; (void)s; (void)diag;
  return false;
#endif
}

std::string RtlProjectManager::BuildModuleId(const std::string& name, size_t ord) {
  std::string s = Slugify(name); return (s.empty() ? "rtl" : s) + "_" + std::to_string(ord);
}

std::string RtlProjectManager::Slugify(const std::string& t) {
  std::string r; for (char ch : t) { if (std::isalnum(static_cast<unsigned char>(ch))) r += static_cast<char>(std::tolower(static_cast<unsigned char>(ch))); else if (ch == ' ') r += '_'; } return r;
}

std::string RtlProjectManager::JoinTempFileArguments(const std::vector<std::string>& paths) {
  std::string r; for (const auto& p : paths) r += " " + LocalEscape(p); return r;
}

} // namespace plc

