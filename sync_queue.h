/***************************************************************
 * Copyright (C) 2025 RoboForce, Inc. All Rights Reserved.
 * Proprietary and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 ***************************************************************/
#pragma once
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

namespace roboforce::driver {
// Thread-safe queue with blocking push and pop operations, and timeout for
// blocking operations
  template < typename T >
  class SyncQueue {
public:
    explicit SyncQueue(size_t max_size = 100);

    // Push item into the queue (blocking with timeout)
    bool push(const T & item, std::chrono::milliseconds timeout);

    // Pop item from the queue (blocking with timeout)
    std::optional < T > pop(std::chrono::milliseconds timeout);

    // Try to pop an item (non-blocking)
    std::optional < T > try_pop();

    // Check if the queue is empty
    bool empty() const;

    // Check the size of the queue
    size_t size() const;

private:
    std::queue < T > queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_push_; // For blocking push operations when full
    std::condition_variable cond_pop_; // For blocking pop operations when empty
    size_t max_size_; // Maximum size of the queue (0 means no limit)
  };
}  // namespace roboforce::driver
