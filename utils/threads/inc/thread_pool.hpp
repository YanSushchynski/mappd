#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>

struct thread_pool : std::vector<std::pair<std::thread, uint32_t>>, std::thread {

  using workers_base_t = std::vector<std::pair<std::thread, uint32_t>>;
  using observer_base_t = std::thread;
  enum struct thread_flag_t : uint32_t { UNDEFINED = 0u, RUNNING, STOPPED };

public:
  explicit thread_pool()
      : stop_(false), notified_(false), observer_base_t(std::thread([this]() -> auto {
          while (true) {
            {
              std::unique_lock<std::mutex> lock(mtx_);
              cv_.wait(lock, [this]() -> bool { return stop_ || notified_; });
            }

            if (stop_) {

              /* Wait all threads */
              for (auto it = this->workers_base_t::begin(); it != this->workers_base_t::end(); it++) {
                it->first.join();
                this->workers_base_t::erase(it);
              }

              /* Stop observer thread */
              return;
            } else if (notified_) {

              for (auto it = this->workers_base_t::begin(); it != this->workers_base_t::end(); it++) {
                if (it->second == static_cast<uint32_t>(thread_flag_t::STOPPED)) {
                  it->first.join();
                  this->workers_base_t::erase(it);
                }
              }

              notified_ = false;
            }
          }
        })) {}

  virtual ~thread_pool() {
    if (!stop_)
      stop();
  }

  void stop() {
    std::unique_lock<std::mutex> lock(mtx_);
    stop_ = true;
    cv_.notify_one();

    /* Wait observer thread */
    this->observer_base_t::join();
  }

  template <typename Function, typename... Args>
  auto push(Function &&f, Args &&... args) -> std::future<decltype(f(args...))> {
    using return_type = decltype(f(args...));
    auto task = std::make_shared<std::packaged_task<return_type(void)>>(
        std::bind(std::forward<Function>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();

    {
      std::unique_lock<std::mutex> lock(mtx_);
      if (stop_)
        throw std::runtime_error("enqueue on stopped thread pool");

      std::shared_ptr<uint32_t> flag_ptr(nullptr);
      this->workers_base_t::push_back({std::thread([this, task, flag_ptr](void) -> void {
                                         (*task)();
                                         *flag_ptr = static_cast<uint32_t>(thread_flag_t::STOPPED);
                                         notified_ = true;
                                         cv_.notify_one();
                                       }),

                                       static_cast<uint32_t>(thread_flag_t::RUNNING)});
      flag_ptr = std::shared_ptr<uint32_t>(&this->workers_base_t::back().second);
    }

    return std::move(res);
  }

  template <typename Class, typename Function, typename... Args>
  auto push(Class *p_obj, Function &&f, Args &&... args) -> std::future<decltype((p_obj->*f)(args...))> {
    using return_type = decltype((p_obj->*f)(args...));
    auto task = std::make_shared<std::packaged_task<return_type(void)>>(
        std::bind(std::forward<Function>(f), p_obj, std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();

    {
      std::unique_lock<std::mutex> lock(mtx_);
      if (stop_)
        throw std::runtime_error("enqueue on stopped thread pool");

      std::shared_ptr<uint32_t> flag_ptr(nullptr);
      this->workers_base_t::push_back({std::thread([this, task, flag_ptr]() -> void {
                                         (*task)();
                                         *flag_ptr = static_cast<uint32_t>(thread_flag_t::STOPPED);
                                         notified_ = true;
                                         cv_.notify_one();
                                       }),

                                       static_cast<uint32_t>(thread_flag_t::RUNNING)});
      flag_ptr = std::shared_ptr<uint32_t>(&this->workers_base_t::back().second);
    }

    return std::move(res);
  }

private:
  mutable std::mutex mtx_;
  mutable std::condition_variable cv_;
  mutable std::atomic<bool> stop_, notified_;
};

#endif /* THREAD_POOL_HPP */
