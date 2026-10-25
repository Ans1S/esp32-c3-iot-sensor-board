#include "filesystem_guard.h"
#include <Arduino.h>
namespace station {
namespace {
SemaphoreHandle_t sharedFilesystemMutex = nullptr;
bool archiveActive = false;
uint32_t archiveTouchedMs = 0;
}
bool initializeFilesystemLock() {
  if (!sharedFilesystemMutex) sharedFilesystemMutex = xSemaphoreCreateMutex();
  return sharedFilesystemMutex != nullptr;
}
SemaphoreHandle_t filesystemMutex() { return sharedFilesystemMutex; }
size_t historyWorkspaceReserve() {
  return 256U*1024U + (archiveActive && uint32_t(millis()-archiveTouchedMs)<60000 ? 64U*1024U : 0);
}
void noteArchiveWork() { archiveActive=true;archiveTouchedMs=millis(); }
}
