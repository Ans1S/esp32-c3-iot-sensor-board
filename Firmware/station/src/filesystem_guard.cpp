#include "filesystem_guard.h"
#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>
#include "sensor_timing.h"
namespace station {
namespace {
SemaphoreHandle_t sharedFilesystemMutex = nullptr;
std::atomic<bool> archiveActive{false};
std::atomic<uint64_t> archiveTouchedMs{0};
std::atomic<uint32_t> spaceEpoch{1};
}
bool initializeFilesystemLock() {
  if (!sharedFilesystemMutex) sharedFilesystemMutex = xSemaphoreCreateMutex();
  return sharedFilesystemMutex != nullptr;
}
SemaphoreHandle_t filesystemMutex() { return sharedFilesystemMutex; }
uint32_t filesystemSpaceEpoch() { return spaceEpoch.load(std::memory_order_acquire); }
void noteFilesystemMutation() { spaceEpoch.fetch_add(1, std::memory_order_release); }
size_t historyWorkspaceReserve() {
  if (!archiveActive.load(std::memory_order_acquire)) return 256U * 1024U;
  const uint64_t now = lil::timing::signedUsToMs(esp_timer_get_time());
  const uint64_t touched = archiveTouchedMs.load(std::memory_order_acquire);
  return 256U * 1024U + (now >= touched && now - touched < 60000ULL ? 64U * 1024U : 0);
}
void noteArchiveWork() {
  archiveTouchedMs.store(lil::timing::signedUsToMs(esp_timer_get_time()),
                         std::memory_order_release);
  archiveActive.store(true, std::memory_order_release);
  noteFilesystemMutation();
}
}
