/***************************************************************
 * Copyright (C) 2025 RoboForce, Inc. All Rights Reserved.
 * Proprietary and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 ***************************************************************/
#include "sync_queue.h"

#include <chrono>
namespace roboforce::driver {
  template < typename T >
  SyncQueue < T > ::SyncQueue(size_t max_size) : max_size_(max_size) {
  }

  template < typename T >
  bool SyncQueue < T > ::push(const T & item, std::chrono::milliseconds timeout) {
    std::unique_lock < std::mutex > lock(mutex_);

    // Wait with timeout for space to become available
    if (!cond_push_.wait_for(
        lock, timeout, [this]() {
      return max_size_ == 0 || queue_.size() < max_size_;
    }))
    {
      return false; // Timeout occurred
    }

    queue_.push(item);
    cond_pop_.notify_one();
    return true;
  }

  template < typename T >
  std::optional < T > SyncQueue < T > ::pop(std::chrono::milliseconds timeout) {
    std::unique_lock < std::mutex > lock(mutex_);

    // Wait with timeout for an item to become available
    if (!cond_pop_.wait_for(
        lock, timeout,
        [this]() {return !queue_.empty();}))
    {
      return std::nullopt; // Timeout occurred
    }

    T item = queue_.front();
    queue_.pop();
    cond_push_.notify_one();
    return item;
  }

  template < typename T >
  std::optional < T > SyncQueue < T > ::try_pop() {
    std::lock_guard < std::mutex > lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt; // No item to pop
    }

    T item = queue_.front();
    queue_.pop();
    cond_push_.notify_one();
    return item;
  }

  template < typename T >
  bool SyncQueue < T > ::empty() const {
    std::lock_guard < std::mutex > lock(mutex_);
    return queue_.empty();
  }

  template < typename T >
  size_t SyncQueue < T > ::size() const {
    std::lock_guard < std::mutex > lock(mutex_);
    return queue_.size();
  }

  // Explicit instantiation for std::vector<uint8_t>
  template class SyncQueue < std::vector < uint8_t >>;
}  // namespace roboforce::driver
