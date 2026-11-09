#ifdef NDEBUG
#error "Regression assertions must be enabled"
#endif

#include <cassert>
#include "thingspeak_account_job.h"

void testRadioTick() { ++testMillis; }

int main() {
  using station::ThingSpeakApiResult;
  using station::ThingSpeakAccountJob;
  using station::ThingSpeakMutationResult;
  using Poll = ThingSpeakAccountJob::Poll;
  using Operation = ThingSpeakAccountJob::Operation;
  ThingSpeakAccountJob list;
  ThingSpeakAccountJob::Request request;
  ThingSpeakApiResult result;
  const auto start = [&list](const char* account, const char* owner, uint32_t now) {
    ThingSpeakAccountJob::Request read;
    snprintf(read.account, sizeof(read.account), "%s", account);
    return list.start(std::move(read), owner, now);
  };

  assert(start("", "session-a", 0) == 0 && !list.take(request));
  const auto first = start("synthetic-account", "session-a", 0);
  assert(first != 0 && start("synthetic-account", "session-a", 1) == first);
  assert(!start("synthetic-account", "session-b", 1));
  assert(list.collect(first, "synthetic-account", "session-a", 1, result) == Poll::Pending);
  assert(list.collect(first, "synthetic-account", "session-b", 1, result) == Poll::Missing);
  assert(list.collect(first + 1, "synthetic-account", "session-a", 1, result) == Poll::Missing);
  assert(list.take(request) && request.ticket == first && !strcmp(request.account, "synthetic-account"));
  assert(!list.take(request));
  assert(list.collect(first, "synthetic-account", "session-a", 2, result) == Poll::Pending);
  assert(!start("replacement-account", "session-a", 2));
  assert(list.expireAndWait(3) == UINT32_MAX);
  list.complete({true, 200, "[{}]", ""}, 25);
  assert(list.collect(first, "replacement-account", "session-a", 26, result) == Poll::Missing);
  assert(start("synthetic-account", "session-a", 26) == first && !list.take(request));
  assert(list.collect(first, "synthetic-account", "session-a", 26, result) == Poll::Ready);
  assert(result.success && result.httpStatus == 200 && !strcmp(result.response.c_str(), "[{}]"));
  assert(list.collect(first, "synthetic-account", "session-a", 26, result) == Poll::Missing);

  const uint32_t nearWrap = UINT32_MAX - 100;
  const auto second = start("synthetic-account", "session-a", nearWrap);
  assert(second != first && list.take(request));
  list.complete({false, 503, "", "Unavailable"}, nearWrap);
  const uint32_t expiry = nearWrap + ThingSpeakAccountJob::kResultLifetimeMs;
  assert(list.expireAndWait(expiry - 1) == 1);
  assert(list.expireAndWait(expiry) == UINT32_MAX);
  assert(list.collect(second, "synthetic-account", "session-a", expiry, result) == Poll::Missing);

  const auto third = start("synthetic-account", "session-a", expiry);
  assert(third != second && list.take(request));
  list.complete({true, 200, "[]", ""}, expiry);
  const auto fourth = start("replacement-account", "session-a", expiry);
  assert(fourth != 0 && fourth != third);
  assert(list.collect(third, "synthetic-account", "session-a", expiry, result) == Poll::Missing);
  assert(list.take(request) && !strcmp(request.account, "replacement-account"));
  list.complete({false, 503, "", "Unavailable"}, expiry);
  assert(list.collect(fourth, "replacement-account", "session-a", expiry, result) == Poll::Ready);
  assert(!result.success && result.httpStatus == 503 && !strcmp(result.error.c_str(), "Unavailable"));
  for (const auto operation : {Operation::QuickCreate, Operation::Create, Operation::Update,
                               Operation::Clear, Operation::Delete}) {
    ThingSpeakAccountJob::Request command;
    command.operation = operation;
    command.channelId = 19;
    command.configResetGeneration = 7;
    snprintf(command.account, sizeof(command.account), "%s", "synthetic-account");
    snprintf(command.name, sizeof(command.name), "%s", "Snapshot");
    command.settings = "name=Snapshot";
    assert(!list.start(command, "session-a", expiry));
    command.nonce = 11;
    const auto id = list.start(command, "session-a", expiry);
    assert(id && list.mutationPending());
    assert(list.take(request) && request.operation == operation && request.nonce == 11 &&
           !strcmp(request.settings.c_str(), "name=Snapshot"));
    assert(list.start(command, "session-a", expiry) == id && !list.take(request));
    command.nonce++;
    assert(!list.start(command, "session-a", expiry));
    list.complete({true, 200, "{}", ""}, expiry);
    ThingSpeakMutationResult final;
    // A browser that never polls cannot discard the remote result before it
    // is applied. The ready receipt's expiry starts after local completion.
    const uint32_t afterWait = expiry + ThingSpeakAccountJob::kResultLifetimeMs + 1;
    assert(list.expireAndWait(afterWait) == UINT32_MAX && list.mutationPending());
    assert(list.collect(id, "synthetic-account", "session-a", afterWait, final) == Poll::Pending);
    assert(list.takeForApply(request, result) && request.channelId == 19 && request.configResetGeneration == 7 &&
           !strcmp(request.name, "Snapshot") && !strcmp(request.account, "synthetic-account") && result.success);
    assert(!list.takeForApply(request, result));
    list.finish({true, "Saved", 19, 2}, afterWait);
    assert(!list.mutationPending());
    assert(list.collect(id, "synthetic-account", "session-a", afterWait, result) == Poll::Missing);
    assert(list.collect(id, "synthetic-account", "session-b", afterWait, final) == Poll::Missing);
    assert(list.collect(id, "synthetic-account", "session-a", afterWait, final) == Poll::Ready);
    assert(final.success && final.channelId == 19 && final.slot == 2);
  }
  puts("ThingSpeak account jobs: deduplication, application without polling, account/session isolation and expiry passed");
}
