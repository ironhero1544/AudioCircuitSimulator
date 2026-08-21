#include "plc_emulator/audio/virtual_audio_installer.h"

#include "plc_emulator/audio/audio_circuit_runtime.h"

#include <algorithm>
#include <cctype>
#include <string>

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

bool ContainsIgnoreCase(std::string text, std::string needle) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::transform(needle.begin(), needle.end(), needle.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return text.find(needle) != std::string::npos;
}

#ifdef _WIN32
std::wstring QuotePowerShellLiteral(const std::wstring& value) {
  std::wstring result = L"'";
  for (wchar_t c : value) {
    result.push_back(c);
    if (c == L'\'') result.push_back(L'\'');
  }
  result.push_back(L'\'');
  return result;
}
#endif

}  // namespace

bool IsVirtualAudioDriverInstalled() {
  const auto& devices = GetAudioOutputDevices(false);
  return std::any_of(devices.begin(), devices.end(),
                     [](const AudioOutputDevice& device) {
                       return ContainsIgnoreCase(device.name,
                                                 "virtual audio cable") &&
                              !ContainsIgnoreCase(device.name, "microphone");
                     });
}

AudioRtlProcessResult InstallVirtualAudioDriver() {
  AudioRtlProcessResult result;
#ifdef _WIN32
  wchar_t temp_path[MAX_PATH] = {};
  if (GetTempPathW(MAX_PATH, temp_path) == 0) {
    result.error = "Cannot resolve the Windows temporary directory.";
    return result;
  }
  const std::wstring script_path =
      std::wstring(temp_path) + L"AudioCircuit-InstallVirtualAudio.ps1";
  const std::wstring log_path =
      std::wstring(temp_path) + L"AudioCircuit-InstallVirtualAudio.log";
  const std::wstring script =
      L"$ErrorActionPreference='Stop'\r\n"
      L"$operation=Join-Path ([IO.Path]::GetTempPath()) "
      L"('AudioCircuit-VAC471-'+[guid]::NewGuid().ToString('N'))\r\n"
      L"$archive=Join-Path $operation 'vac471lite.zip'\r\n"
      L"$unpack=Join-Path $operation 'vac471lite'\r\n"
      L"New-Item -ItemType Directory -Path $operation | Out-Null\r\n"
      L"try {\r\n"
      L"  $brokenMttDevices=@(Get-PnpDevice -Class Media -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like 'Virtual Audio Driver*'})\r\n"
      L"  if($brokenMttDevices.Count -gt 0){\r\n"
      L"    $cleanup=Start-Process -FilePath (Join-Path $env:SystemRoot 'System32\\pnputil.exe') -ArgumentList '/remove-device /deviceid \"ROOT\\VirtualAudioDriver\"' -WindowStyle Hidden -Wait -PassThru\r\n"
      L"    if(($cleanup.ExitCode -ne 0) -and ($cleanup.ExitCode -ne 3010)){throw ('Cannot remove the obsolete MTT devices; pnputil exited with '+$cleanup.ExitCode)}\r\n"
      L"  }\r\n"
      L"  $readyVac=Get-PnpDevice -Class Media -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like '*Virtual Audio Cable*' -and $_.Status -eq 'OK'} | Select-Object -First 1\r\n"
      L"  if($readyVac){('Virtual Audio Cable is already installed. Device: '+$readyVac.FriendlyName) | Set-Content -LiteralPath " + QuotePowerShellLiteral(log_path) + L" -Encoding UTF8; exit 0}\r\n"
      L"  Invoke-WebRequest -UseBasicParsing -Uri 'https://software.muzychenko.net/freeware/vac471lite.zip' -OutFile $archive\r\n"
      L"  $archiveHash=(Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash\r\n"
      L"  if($archiveHash -ne '1ED42A7E763BF6237438FB62191AE88B2922F014B37658B29D7801D72C9D5357'){throw ('VAC package hash mismatch: '+$archiveHash)}\r\n"
      L"  Expand-Archive -LiteralPath $archive -DestinationPath $unpack\r\n"
      L"  $setup=Join-Path $unpack 'setup.exe'\r\n"
      L"  $catalog=Join-Path $unpack 'vrtaucbl.cat'\r\n"
      L"  if((-not (Test-Path -LiteralPath $setup)) -or (-not (Test-Path -LiteralPath $catalog))){throw 'The VAC package is incomplete'}\r\n"
      L"  $setupSignature=Get-AuthenticodeSignature -LiteralPath $setup\r\n"
      L"  $catalogSignature=Get-AuthenticodeSignature -LiteralPath $catalog\r\n"
      L"  if($setupSignature.Status -ne 'Valid'){throw ('VAC setup signature is '+$setupSignature.Status)}\r\n"
      L"  if(($catalogSignature.Status -ne 'Valid') -or ($catalogSignature.SignerCertificate.Subject -notlike '*Microsoft Windows Hardware Compatibility Publisher*')){throw 'VAC driver catalog is not Microsoft hardware-signed'}\r\n"
      L"  $process=Start-Process -FilePath $setup -WorkingDirectory $unpack -Wait -PassThru\r\n"
      L"  if(($process.ExitCode -ne 0) -and ($process.ExitCode -ne 3010)){throw ('VAC installer exited with '+$process.ExitCode)}\r\n"
      L"  $device=$null\r\n"
      L"  for($attempt=0;($attempt -lt 20) -and (-not $device);$attempt++){Start-Sleep -Milliseconds 500; $device=Get-PnpDevice -Class Media -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like '*Virtual Audio Cable*' -and $_.Status -eq 'OK'} | Select-Object -First 1}\r\n"
      L"  if(-not $device){throw 'VAC setup closed, but Windows did not create an active Virtual Audio Cable device'}\r\n"
      L"  $message='Virtual Audio Cable 4.71 Lite installation completed.'\r\n"
      L"  $message+=' Device: '+$device.FriendlyName\r\n"
      L"  $message | "
      L"Set-Content -LiteralPath " + QuotePowerShellLiteral(log_path) +
      L" -Encoding UTF8\r\n"
      L"  exit 0\r\n"
      L"} catch { $_.Exception.Message | Set-Content -LiteralPath " +
      QuotePowerShellLiteral(log_path) +
      L" -Encoding UTF8; exit 1 } finally { if(Test-Path -LiteralPath $operation){Remove-Item -LiteralPath $operation -Recurse -Force} }\r\n";

  HANDLE script_file = CreateFileW(script_path.c_str(), GENERIC_WRITE, 0,
                                   nullptr, CREATE_ALWAYS,
                                   FILE_ATTRIBUTE_NORMAL, nullptr);
  if (script_file == INVALID_HANDLE_VALUE) {
    result.error = "Cannot create the virtual audio installer script.";
    return result;
  }
  const unsigned char bom[] = {0xff, 0xfe};
  DWORD written = 0;
  WriteFile(script_file, bom, sizeof(bom), &written, nullptr);
  WriteFile(script_file, script.data(),
            static_cast<DWORD>(script.size() * sizeof(wchar_t)), &written,
            nullptr);
  CloseHandle(script_file);

  const std::wstring parameters =
      L"-NoProfile -ExecutionPolicy Bypass -File \"" + script_path + L"\"";
  SHELLEXECUTEINFOW execute{};
  execute.cbSize = sizeof(execute);
  execute.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_FLAG_NO_UI;
  execute.lpVerb = L"runas";
  execute.lpFile = L"powershell.exe";
  execute.lpParameters = parameters.c_str();
  execute.nShow = SW_HIDE;

  if (!ShellExecuteExW(&execute) || execute.hProcess == nullptr) {
    const DWORD error = GetLastError();
    DeleteFileW(script_path.c_str());
    result.error = error == ERROR_CANCELLED
                       ? "Virtual audio installation was cancelled at the administrator prompt."
                       : "Cannot launch the virtual audio installer. Windows error " +
                             std::to_string(error) + ".";
    return result;
  }

  result.started = true;
  const DWORD wait = WaitForSingleObject(execute.hProcess, 15 * 60 * 1000);
  if (wait == WAIT_TIMEOUT) {
    result.timed_out = true;
    result.error = "Virtual audio installation timed out.";
    TerminateProcess(execute.hProcess, ERROR_TIMEOUT);
  }
  DWORD exit_code = 1;
  GetExitCodeProcess(execute.hProcess, &exit_code);
  result.exit_code = exit_code;
  CloseHandle(execute.hProcess);
  DeleteFileW(script_path.c_str());

  HANDLE log_file = CreateFileW(log_path.c_str(), GENERIC_READ,
                                FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (log_file != INVALID_HANDLE_VALUE) {
    const DWORD size = GetFileSize(log_file, nullptr);
    std::string bytes(size, '\0');
    DWORD read = 0;
    if (size > 0) ReadFile(log_file, bytes.data(), size, &read, nullptr);
    bytes.resize(read);
    if (bytes.size() >= 3 &&
        static_cast<unsigned char>(bytes[0]) == 0xef &&
        static_cast<unsigned char>(bytes[1]) == 0xbb &&
        static_cast<unsigned char>(bytes[2]) == 0xbf) {
      bytes.erase(0, 3);
    }
    result.output = bytes;
    CloseHandle(log_file);
    DeleteFileW(log_path.c_str());
  }
  if (!result.Succeeded() && result.error.empty()) {
    result.error = result.output.empty()
                       ? "VAC installation failed, was cancelled, or its setup window was closed before installation completed."
                       : result.output;
  }
#else
  result.error = "Virtual audio installation is available only on Windows.";
#endif
  return result;
}


AudioRtlProcessResult RemoveVirtualAudioDriver() {
  AudioRtlProcessResult result;
#ifdef _WIN32
  wchar_t temp_path[MAX_PATH] = {};
  if (GetTempPathW(MAX_PATH, temp_path) == 0) {
    result.error = "Cannot resolve the Windows temporary directory.";
    return result;
  }

  const std::wstring script_path =
      std::wstring(temp_path) + L"AudioCircuit-RemoveVirtualAudio.ps1";
  const std::wstring log_path =
      std::wstring(temp_path) + L"AudioCircuit-RemoveVirtualAudio.log";

  // Prefer VAC's own uninstaller. If the installed setup executable is
  // missing or leaves the driver behind, fall back to PnPUtil/service cleanup.
  const std::wstring script =
      L"$ErrorActionPreference='Stop'\r\n"
      L"$log=" + QuotePowerShellLiteral(log_path) + L"\r\n"
      L"function Write-Result([string]$message){$message | Set-Content -LiteralPath $log -Encoding UTF8}\r\n"
      L"try {\r\n"
      L"  $vacDevices=@(Get-PnpDevice -Class Media -PresentOnly -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like '*Virtual Audio Cable*'})\r\n"
      L"  $driverInfs=@()\r\n"
      L"  foreach($device in $vacDevices){try{$prop=Get-PnpDeviceProperty -InstanceId $device.InstanceId -KeyName 'DEVPKEY_Device_DriverInfPath' -ErrorAction Stop; if($prop.Data){$driverInfs+=[string]$prop.Data}}catch{}}\r\n"
      L"  $registryRoots=@('HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\*','HKLM:\\SOFTWARE\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\*')\r\n"
      L"  $vacEntries=@()\r\n"
      L"  foreach($root in $registryRoots){$vacEntries+=@(Get-ItemProperty -Path $root -ErrorAction SilentlyContinue | Where-Object {$_.DisplayName -like 'Virtual Audio Cable*'})}\r\n"
      L"  $setupCandidates=@()\r\n"
      L"  foreach($entry in $vacEntries){\r\n"
      L"    if($entry.InstallLocation){$setupCandidates+=(Join-Path ([string]$entry.InstallLocation) 'setup.exe')}\r\n"
      L"    if($entry.UninstallString){$u=[string]$entry.UninstallString; if($u -match '^\\s*\"([^\"]+\\.exe)\"'){$setupCandidates+=$matches[1]} elseif($u -match '^\\s*([^\\s]+\\.exe)'){$setupCandidates+=$matches[1]}}\r\n"
      L"  }\r\n"
      L"  if($env:ProgramFiles){$setupCandidates+=(Join-Path $env:ProgramFiles 'Virtual Audio Cable\\setup.exe')}\r\n"
      L"  if(${env:ProgramFiles(x86)}){$setupCandidates+=(Join-Path ${env:ProgramFiles(x86)} 'Virtual Audio Cable\\setup.exe')}\r\n"
      L"  $setup=$setupCandidates | Where-Object {$_ -and (Test-Path -LiteralPath $_)} | Select-Object -Unique | Select-Object -First 1\r\n"
      L"  if(($vacDevices.Count -eq 0) -and (-not $setup) -and ($vacEntries.Count -eq 0)){Write-Result 'Virtual Audio Cable is not installed. Nothing was removed.'; exit 0}\r\n"
      L"  $usedOfficial=$false\r\n"
      L"  $rebootRequired=$false\r\n"
      L"  if($setup){\r\n"
      L"    $usedOfficial=$true\r\n"
      L"    $process=Start-Process -FilePath $setup -ArgumentList '-u' -Wait -PassThru\r\n"
      L"    if(($process.ExitCode -ne 0) -and ($process.ExitCode -ne 3010)){throw ('VAC uninstaller exited with '+$process.ExitCode)}\r\n"
      L"    if($process.ExitCode -eq 3010){$rebootRequired=$true}\r\n"
      L"  }\r\n"
      L"  for($attempt=0;$attempt -lt 20;$attempt++){\r\n"
      L"    $remaining=@(Get-PnpDevice -Class Media -PresentOnly -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like '*Virtual Audio Cable*'})\r\n"
      L"    if($remaining.Count -eq 0){break}\r\n"
      L"    Start-Sleep -Milliseconds 500\r\n"
      L"  }\r\n"
      L"  $remaining=@(Get-PnpDevice -Class Media -PresentOnly -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like '*Virtual Audio Cable*'})\r\n"
      L"  if((-not $usedOfficial) -or ($remaining.Count -gt 0)){\r\n"
      L"    foreach($device in $remaining){\r\n"
      L"      try{$prop=Get-PnpDeviceProperty -InstanceId $device.InstanceId -KeyName 'DEVPKEY_Device_DriverInfPath' -ErrorAction Stop; if($prop.Data){$driverInfs+=[string]$prop.Data}}catch{}\r\n"
      L"      & (Join-Path $env:SystemRoot 'System32\\pnputil.exe') /remove-device $device.InstanceId | Out-Null\r\n"
      L"      if(($LASTEXITCODE -ne 0) -and ($LASTEXITCODE -ne 3010)){throw ('pnputil could not remove VAC device '+$device.InstanceId+'; exit '+$LASTEXITCODE)}\r\n"
      L"      if($LASTEXITCODE -eq 3010){$rebootRequired=$true}\r\n"
      L"    }\r\n"
      L"    foreach($inf in @($driverInfs | Where-Object {$_} | Select-Object -Unique)){\r\n"
      L"      & (Join-Path $env:SystemRoot 'System32\\pnputil.exe') /delete-driver $inf /uninstall /force | Out-Null\r\n"
      L"      if(($LASTEXITCODE -ne 0) -and ($LASTEXITCODE -ne 3010)){throw ('pnputil could not delete VAC driver package '+$inf+'; exit '+$LASTEXITCODE)}\r\n"
      L"      if($LASTEXITCODE -eq 3010){$rebootRequired=$true}\r\n"
      L"    }\r\n"
      L"    $vacServices=@(Get-CimInstance Win32_SystemDriver -ErrorAction SilentlyContinue | Where-Object {$_.PathName -like '*vrtaucbl.sys*' -or $_.Name -like 'VirtualAudioCable*'})\r\n"
      L"    foreach($svc in $vacServices){& (Join-Path $env:SystemRoot 'System32\\sc.exe') stop $svc.Name | Out-Null; & (Join-Path $env:SystemRoot 'System32\\sc.exe') delete $svc.Name | Out-Null}\r\n"
      L"    foreach($entry in $vacEntries){try{Remove-Item -LiteralPath $entry.PSPath -Recurse -Force -ErrorAction Stop}catch{}}\r\n"
      L"    $folders=@(); if($env:ProgramFiles){$folders+=(Join-Path $env:ProgramFiles 'Virtual Audio Cable')}; if(${env:ProgramFiles(x86)}){$folders+=(Join-Path ${env:ProgramFiles(x86)} 'Virtual Audio Cable')}\r\n"
      L"    foreach($folder in $folders){if(Test-Path -LiteralPath $folder){try{Remove-Item -LiteralPath $folder -Recurse -Force -ErrorAction Stop}catch{$rebootRequired=$true}}}\r\n"
      L"  }\r\n"
      L"  $remaining=@(Get-PnpDevice -Class Media -PresentOnly -ErrorAction SilentlyContinue | Where-Object {$_.FriendlyName -like '*Virtual Audio Cable*'})\r\n"
      L"  if($remaining.Count -gt 0){throw 'VAC removal finished, but a Virtual Audio Cable device is still present. Close applications using VAC and restart Windows, then run Delete Tools again.'}\r\n"
      L"  $message='Virtual Audio Cable driver removed.'\r\n"
      L"  if($rebootRequired){$message+=' Windows restart is required to finish cleanup.'}\r\n"
      L"  Write-Result $message\r\n"
      L"  exit 0\r\n"
      L"} catch { $_.Exception.Message | Set-Content -LiteralPath $log -Encoding UTF8; exit 1 }\r\n";

  HANDLE script_file = CreateFileW(script_path.c_str(), GENERIC_WRITE, 0,
                                   nullptr, CREATE_ALWAYS,
                                   FILE_ATTRIBUTE_NORMAL, nullptr);
  if (script_file == INVALID_HANDLE_VALUE) {
    result.error = "Cannot create the virtual audio removal script.";
    return result;
  }
  const unsigned char bom[] = {0xff, 0xfe};
  DWORD written = 0;
  WriteFile(script_file, bom, sizeof(bom), &written, nullptr);
  WriteFile(script_file, script.data(),
            static_cast<DWORD>(script.size() * sizeof(wchar_t)), &written,
            nullptr);
  CloseHandle(script_file);

  const std::wstring parameters =
      L"-NoProfile -ExecutionPolicy Bypass -File \"" + script_path + L"\"";
  SHELLEXECUTEINFOW execute{};
  execute.cbSize = sizeof(execute);
  execute.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_FLAG_NO_UI;
  execute.lpVerb = L"runas";
  execute.lpFile = L"powershell.exe";
  execute.lpParameters = parameters.c_str();
  execute.nShow = SW_HIDE;

  if (!ShellExecuteExW(&execute) || execute.hProcess == nullptr) {
    const DWORD error = GetLastError();
    DeleteFileW(script_path.c_str());
    result.error = error == ERROR_CANCELLED
                       ? "Virtual audio removal was cancelled at the administrator prompt."
                       : "Cannot launch the virtual audio remover. Windows error " +
                             std::to_string(error) + ".";
    return result;
  }

  result.started = true;
  const DWORD wait = WaitForSingleObject(execute.hProcess, 15 * 60 * 1000);
  if (wait == WAIT_TIMEOUT) {
    result.timed_out = true;
    result.error = "Virtual audio removal timed out.";
    TerminateProcess(execute.hProcess, ERROR_TIMEOUT);
  }
  DWORD exit_code = 1;
  GetExitCodeProcess(execute.hProcess, &exit_code);
  result.exit_code = exit_code;
  CloseHandle(execute.hProcess);
  DeleteFileW(script_path.c_str());

  HANDLE log_file = CreateFileW(log_path.c_str(), GENERIC_READ,
                                FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (log_file != INVALID_HANDLE_VALUE) {
    const DWORD size = GetFileSize(log_file, nullptr);
    std::string bytes(size, '\0');
    DWORD read = 0;
    if (size > 0) ReadFile(log_file, bytes.data(), size, &read, nullptr);
    bytes.resize(read);
    if (bytes.size() >= 3 &&
        static_cast<unsigned char>(bytes[0]) == 0xef &&
        static_cast<unsigned char>(bytes[1]) == 0xbb &&
        static_cast<unsigned char>(bytes[2]) == 0xbf) {
      bytes.erase(0, 3);
    }
    result.output = bytes;
    CloseHandle(log_file);
    DeleteFileW(log_path.c_str());
  }
  if (!result.Succeeded() && result.error.empty()) {
    result.error = result.output.empty()
                       ? "VAC removal failed or was cancelled."
                       : result.output;
  }
#else
  result.error = "Virtual audio removal is available only on Windows.";
#endif
  return result;
}

}  // namespace plc
