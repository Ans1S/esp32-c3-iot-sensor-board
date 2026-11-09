#pragma once

#include <Arduino.h>
#include <cstring>
#include <utility>
#include <variant>

#include "app_config.h"

namespace station {

struct ThingSpeakApiResult {
  bool success = false;
  int httpStatus = 0;
  String response;
  String error;
};

struct ThingSpeakMutationResult {
  bool success = false;
  String message;
  uint32_t channelId = 0;
  int slot = -1;
};

// The ThingSpeak queue mutex serializes the worker and web task mailbox.
class ThingSpeakAccountJob {
 public:
  enum class Poll { Missing, Pending, Ready };
  enum class Operation { List, QuickCreate, Create, Update, Clear, Delete };
  static constexpr uint32_t kResultLifetimeMs = 30000;
  struct Request {
    Operation operation = Operation::List;
    uint32_t ticket = 0;
    uint32_t nonce = 0;
    uint32_t channelId = 0;
    uint32_t configResetGeneration = 0;
    char account[sizeof(StationConfig::thingSpeakUserApiKey)]{};
    char name[sizeof(ThingSpeakChannelProfile::name)]{};
    String settings;
  };

  uint32_t start(Request request, const char* owner, uint32_t now) {
    expireAndWait(now);
    const size_t ownerLength = strlen(owner);
    if (request.account[0] == '\0' || ownerLength >= sizeof(owner_) ||
        (request.operation != Operation::List && request.nonce == 0)) return 0;
    if (state_ != State::Idle) {
      const bool sameAccount = !strcmp(request.account, request_.account);
      if (sameAccount && !strcmp(owner, owner_) && request.operation == request_.operation &&
          (request.operation == Operation::List || request.nonce == request_.nonce)) return request_.ticket;
      if (state_ != State::Ready || sameAccount) return 0;
      // The station now uses a different account. Drop the obsolete result.
      clear();
    }
    if (++sequence_ == 0) ++sequence_;
    request.ticket = sequence_;
    request_ = std::move(request);
    memcpy(owner_, owner, ownerLength + 1);
    state_ = State::Queued;
    return request_.ticket;
  }

  bool take(Request& request) {
    if (state_ != State::Queued) return false;
    // The settings payload belongs to the network worker. Keep the fixed
    // metadata for authorization and applying the result on the web task.
    request = std::move(request_);
    state_ = State::Running;
    return true;
  }

  void complete(ThingSpeakApiResult result, uint32_t now) {
    result_.emplace<ThingSpeakApiResult>(std::move(result));
    if (request_.operation == Operation::List) {
      finishedMs_ = now;
      state_ = State::Ready;
    } else {
      state_ = State::RemoteDone;
    }
  }

  bool takeForApply(Request& request, ThingSpeakApiResult& result) {
    if (state_ != State::RemoteDone) return false;
    request = std::move(request_);
    result = std::move(std::get<ThingSpeakApiResult>(result_));
    result_.emplace<std::monostate>();
    state_ = State::Applying;
    return true;
  }

  void finish(ThingSpeakMutationResult result, uint32_t now) {
    result_.emplace<ThingSpeakMutationResult>(std::move(result));
    finishedMs_ = now;
    state_ = State::Ready;
  }

  bool mutationPending() const {
    return state_ != State::Idle && state_ != State::Ready && request_.operation != Operation::List;
  }

  template <typename Result>
  Poll collect(uint32_t ticket, const char* account, const char* owner,
               uint32_t now, Result& result) {
    expireAndWait(now);
    if (state_ == State::Idle || ticket != request_.ticket ||
        strcmp(account, request_.account) || strcmp(owner, owner_)) return Poll::Missing;
    if (state_ != State::Ready) return Poll::Pending;
    auto* value = std::get_if<Result>(&result_);
    if (!value) return Poll::Missing;
    result = std::move(*value);
    clear();
    return Poll::Ready;
  }

  uint32_t expireAndWait(uint32_t now) {
    if (state_ != State::Ready) return UINT32_MAX;
    const uint32_t age = now - finishedMs_;
    if (age < kResultLifetimeMs) return kResultLifetimeMs - age;
    clear();
    return UINT32_MAX;
  }

 private:
  enum class State { Idle, Queued, Running, RemoteDone, Applying, Ready };

  void clear() {
    result_.emplace<std::monostate>();
    request_ = {};
    memset(owner_, 0, sizeof(owner_));
    state_ = State::Idle;
  }

  State state_ = State::Idle;
  Request request_{};
  char owner_[33]{};
  uint32_t sequence_ = 0, finishedMs_ = 0;
  std::variant<std::monostate, ThingSpeakApiResult, ThingSpeakMutationResult> result_;
};

}  // namespace station
