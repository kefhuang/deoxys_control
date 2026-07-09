#pragma once

#include <atomic>
#include <string>
#include <utility>

#include <franka/exception.h>

namespace control_exception_utils {

struct ControlCallResult {
  bool succeeded;
  std::string error_message;
};

template <typename Callable>
ControlCallResult RunWithShutdownOnFrankaException(
    Callable&& control_call,
    std::atomic_bool& running,
    std::atomic_bool& termination) {
  try {
    std::forward<Callable>(control_call)();
  } catch (const franka::Exception& exception) {
    running = false;
    termination = true;
    return {false, exception.what()};
  }
  return {true, ""};
}

}  // namespace control_exception_utils
