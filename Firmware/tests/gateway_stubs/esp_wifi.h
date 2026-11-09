#pragma once
#include <cassert>
#include "esp_now.h"
struct TestEsp { void restart() { assert(false); } };
inline TestEsp ESP;
