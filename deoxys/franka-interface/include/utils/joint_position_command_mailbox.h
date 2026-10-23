#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_JOINT_POSITION_COMMAND_MAILBOX_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_JOINT_POSITION_COMMAND_MAILBOX_H_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace control_utils {

struct JointPositionCommand {
  std::array<double, 7> positions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> velocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double time_fraction{1.0};
  std::uint64_t sequence{0};
  bool is_delta{false};
};

// Fixed-capacity single-producer/single-consumer queue. The ZMQ policy thread
// is the only producer and the libfranka callback is the only consumer. Slot
// publication uses release/acquire ordering so a consumer never observes a
// partially written seven-joint target.
template <std::size_t Capacity> class JointPositionCommandMailbox {
  static_assert(Capacity >= 2, "SPSC mailbox needs at least two slots");

public:
  bool Push(JointPositionCommand command) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = Increment(head);
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    command.sequence = next_sequence_++;
    slots_[head] = command;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Consume all currently published entries in one bounded operation and
  // return only the newest command. This is appropriate for streamed targets:
  // replaying stale intermediate goals would add latency after scheduler jitter.
  bool PopLatest(JointPositionCommand &command) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {
      return false;
    }

    const std::size_t latest = head == 0 ? Capacity - 1 : head - 1;
    command = slots_[latest];
    tail_.store(head, std::memory_order_release);
    return true;
  }

  void DiscardAll() {
    const std::size_t head = head_.load(std::memory_order_acquire);
    tail_.store(head, std::memory_order_release);
  }

  std::uint64_t dropped() const {
    return dropped_.load(std::memory_order_relaxed);
  }

  bool IsLockFree() const {
    return head_.is_lock_free() && tail_.is_lock_free() &&
           dropped_.is_lock_free();
  }

private:
  static std::size_t Increment(std::size_t index) {
    return index + 1 == Capacity ? 0 : index + 1;
  }

  std::array<JointPositionCommand, Capacity> slots_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::uint64_t next_sequence_{1};
};

} // namespace control_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_JOINT_POSITION_COMMAND_MAILBOX_H_
