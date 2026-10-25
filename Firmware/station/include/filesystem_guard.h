#pragma once
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
namespace station {
bool initializeFilesystemLock();
SemaphoreHandle_t filesystemMutex();
size_t historyWorkspaceReserve();
void noteArchiveWork();
class FilesystemGuard {
 public:
  FilesystemGuard() : mutex_(filesystemMutex()) { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
  ~FilesystemGuard() { if (mutex_) xSemaphoreGive(mutex_); }
  explicit operator bool() const { return mutex_ != nullptr; }
 private:
  SemaphoreHandle_t mutex_;
};
}
