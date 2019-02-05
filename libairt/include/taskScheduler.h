#pragma once

#include <functional>
#include <chrono>
#include <deque>

namespace airt
{

/**
 * 
 * \class TaskScheduler
 * Very simple task scheduler, that accepts tasks and a delay, and executes them after the delay has passed.
 * \warning The timing is not very precise.
 */
class TaskScheduler
{
public:
  TaskScheduler() = default;
  /**
   * Schedule a task for later execution. 
   * \warning It is not very precise
   * 
   */
  void schedule(size_t wait_ms, std::function<void(void)> task);

  /**
   * Cancel all scheduled tasks
   */
  void cancelAllScheduledTasks();

  /**
     * Call this function to execute the pending tasks (the more frequently you call this function, the more precise
     * the timing will be (but the more CPU will go to waste)
     * \return true if some task was executed
     */
  bool run();

private:
  typedef std::pair<std::chrono::time_point<std::chrono::steady_clock>, std::function<void(void)>> ScheduledTask;
  std::deque<ScheduledTask> tasks;
};
}; // namespace airt