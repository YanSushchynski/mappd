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

template <uint32_t num_threads = 1u>
struct thread_pool : public std::queue<std::function<void(void)>>, public std::vector<std::thread> {
  using tasks_base_t = std::queue<std::function<void(void)>>;
  using workers_base_t = std::vector<std::thread>;

public:
  explicit thread_pool() : stop_(false) {
    for (uint32_t i = 0; i < num_threads; i++) {
      this->workers_base_t::emplace_back([this]() -> auto {
        while (true) {
          std::function<void(void)> task;

          {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this]() -> bool { return stop_ || !this->tasks_base_t::empty(); });

            if (stop_ && this->tasks_base_t::empty())
              return;

            task = std::move(this->tasks_base_t::front());
            this->pop();
          }

          task();
        }
      });
    }
  }

  virtual ~thread_pool() {
    if (!stop_)
      stop();
  }

  void stop() {
    {
      std::unique_lock<std::mutex> lock(mtx_);
      stop_ = true;
    }

    cv_.notify_all();
    for (std::thread &worker : *static_cast<workers_base_t *>(this)) {
      worker.join();
    }
  }

  template <typename Function, typename... Args>
  auto push(Function f, Args... args) -> std::future<decltype(f(args...))> {
    using return_type = decltype(f(args...));
    auto task = std::make_shared<std::packaged_task<return_type(void)>>(
        std::bind(std::forward<Function>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();

    {
      std::unique_lock<std::mutex> lock(mtx_);
      if (stop_)
        throw std::runtime_error("enqueue on stopped thread pool");

      this->tasks_base_t::emplace([task]() -> void { (*task)(); });
    }

    cv_.notify_one();
    return std::move(res);
  }

  template <typename Class, typename Function, typename... Args>
  auto push(Class *p_obj, Function f, Args... args) -> std::future<decltype((p_obj->*f)(args...))> {
    using return_type = decltype((p_obj->*f)(args...));
    auto task = std::make_shared<std::packaged_task<return_type(void)>>(
        std::bind(std::forward<Function>(f), p_obj, std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();

    {
      std::unique_lock<std::mutex> lock(mtx_);
      if (stop_)
        throw std::runtime_error("enqueue on stopped thread pool");

      this->tasks_base_t::emplace([task]() -> void { (*task)(); });
    }

    cv_.notify_one();
    return std::move(res);
  }

private:
  mutable std::mutex mtx_;
  mutable std::condition_variable cv_;
  mutable std::atomic<bool> stop_;
};

#endif /* THREAD_POOL_HPP */
