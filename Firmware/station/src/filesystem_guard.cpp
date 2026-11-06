#include "filesystem_guard.h"
#include <Arduino.h>
#include <esp_timer.h>
namespace station {
namespace {
SemaphoreHandle_t sharedFilesystemMutex = nullptr;
bool archiveActive = false;
uint64_t archiveTouchedMs = 0;
}
bool initializeFilesystemLock() {
  if (!sharedFilesystemMutex) sharedFilesystemMutex = xSemaphoreCreateMutex();
  return sharedFilesystemMutex != nullptr;
}
SemaphoreHandle_t filesystemMutex() { return sharedFilesystemMutex; }
size_t historyWorkspaceReserve() {
  const uint64_t now = static_cast<uint64_t>(esp_timer_get_time()) / 1000;
  return 256U*1024U + (archiveActive && now - archiveTouchedMs < 60000 ? 64U*1024U : 0);
}
void noteArchiveWork() { archiveActive=true;archiveTouchedMs=static_cast<uint64_t>(esp_timer_get_time()) / 1000; }
}
