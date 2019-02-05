#include "taskScheduler.h"
#include "log.h"

using airt::TaskScheduler;

void TaskScheduler::schedule(size_t wait_ms, std::function<void(void)> task)
{
    auto p = std::pair<std::chrono::time_point<std::chrono::steady_clock>, std::function<void(void)>>(std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms), task);
    tasks.push_back(p);
    std::sort(tasks.begin(), tasks.end(), [](const ScheduledTask &a, const ScheduledTask &b) { return a.first < b.first; });
}

bool TaskScheduler::run()
{
    if (!tasks.empty())
    {
        if (tasks.front().first < std::chrono::steady_clock::now())
        {
            Log::info("Executing scheduled task");
            tasks.front().second();
            tasks.pop_front();
            return true;
        }
    }
    return false;
}

void TaskScheduler::cancelAllScheduledTasks() {
    tasks.clear();
}