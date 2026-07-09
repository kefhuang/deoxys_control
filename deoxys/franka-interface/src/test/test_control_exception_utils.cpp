#include <atomic>
#include <iostream>
#include <string>
#include <thread>

#include <franka/exception.h>

#include "utils/control_exception_utils.h"

int main() {
  std::atomic_bool running{true};
  std::atomic_bool termination{false};

  std::thread worker([&termination]() {
    while (!termination) {
      std::this_thread::yield();
    }
  });

  const auto failure =
      control_exception_utils::RunWithShutdownOnFrankaException(
          []() { throw franka::Exception("sentinel control failure"); },
          running,
          termination);

  if (worker.joinable()) {
    worker.join();
  }

  if (failure.succeeded) {
    std::cerr << "throwing call unexpectedly succeeded" << std::endl;
    return 1;
  }
  if (failure.error_message != "sentinel control failure") {
    std::cerr << "exception message was not preserved" << std::endl;
    return 1;
  }
  if (running || !termination) {
    std::cerr << "shutdown flags were not updated" << std::endl;
    return 1;
  }

  running = true;
  termination = false;
  const auto success =
      control_exception_utils::RunWithShutdownOnFrankaException(
          []() {}, running, termination);
  if (!success.succeeded || !success.error_message.empty()) {
    std::cerr << "non-throwing call did not report success" << std::endl;
    return 1;
  }
  if (!running || termination) {
    std::cerr << "successful call changed shutdown flags" << std::endl;
    return 1;
  }

  return 0;
}
