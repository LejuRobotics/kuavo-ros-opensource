#include "hardware_node/sg100_initialization_retry.h"
#include <atomic>
#include <future>
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  // A late device succeeds on the third attempt; success stops all retries.
  {
    sg100::InitializationRetry retry;
    std::atomic<int> attempts{0};
    std::promise<void> recovered;
    auto done = recovered.get_future();
    retry.start(15ms, [&] {
      if (++attempts == 3) { recovered.set_value(); return true; }
      return false;
    }, [] { return false; });
    require(done.wait_for(2s) == std::future_status::ready, "late device never recovered");
    std::this_thread::sleep_for(60ms);
    retry.stop();
    require(attempts == 3, "successful device was initialized again");
  }
  // Stop interrupts the full production delay rather than waiting five seconds.
  {
    sg100::InitializationRetry retry;
    std::atomic<int> attempts{0};
    retry.start(5s, [&] { ++attempts; return false; }, [] { return false; });
    const auto start = std::chrono::steady_clock::now();
    retry.stop();
    require(std::chrono::steady_clock::now() - start < 1s, "stop waited for retry delay");
    require(attempts == 0, "stop caused a new attempt");
  }
  // Cancellation during an attempt must prevent publication and further attempts;
  // stop waits for that attempt to release its temporary device.
  {
    sg100::InitializationRetry retry;
    std::atomic<bool> cancelled{false}, released{false}, ready{false};
    std::atomic<int> attempts{0};
    std::promise<void> entered, release;
    auto entry = entered.get_future();
    auto gate = release.get_future();
    retry.start(1ms, [&] {
      ++attempts;
      entered.set_value();
      gate.wait();
      ready = !cancelled;
      released = true;
      return bool(ready);
    }, [&] { return bool(cancelled); });
    require(entry.wait_for(2s) == std::future_status::ready, "attempt did not start");
    cancelled = true;
    auto stopped = std::async(std::launch::async, [&] { retry.stop(); });
    require(stopped.wait_for(20ms) == std::future_status::timeout, "stop did not join attempt");
    release.set_value();
    stopped.get();
    require(released && !ready && attempts == 1, "cancelled attempt resumed control");
  }
  // An external robot-stop latch also prevents attempts without calling stop yet.
  {
    sg100::InitializationRetry retry;
    std::atomic<int> attempts{0};
    retry.start(1ms, [&] { ++attempts; return false; }, [] { return true; });
    std::this_thread::sleep_for(30ms);
    retry.stop();
    require(attempts == 0, "attempt started after robot stop");
  }
  std::cout << "SG100 retry recovery, cancellation and shutdown tests passed\n";
}
