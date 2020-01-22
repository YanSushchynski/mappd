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

struct thread_pool : std::queue<std::function<void(void)>>, std::vector<std::thread>, std::thread {
  using tasks_base_t = std::queue<std::function<void(void)>>;
  using workers_base_t = std::vector<std::thread>;
  using observer_base_t = std::thread;

public:
  explicit thread_pool()
      : stop_(false), observer_base_t(std::thread([this]() -> auto {
          while (true) {
            {
              std::unique_lock<std::mutex> lock(mtx_);
              cv_.wait(lock, [this]() -> bool { return stop_ || !this->tasks_base_t::empty(); });
            }

            if (stop_ && this->tasks_base_t::empty()) {

			  /* Thread pool stopped, terminate observer thread */
              return;
            } else if (!this->tasks_base_t::empty()){

              /* If task queue isn't empty */
			  /* Run left tasks */
              while (!this->tasks_base_t::empty()) {

                std::shared_ptr<std::thread> thr_ptr = std::shared_ptr<std::thread>(
                    new std::thread([this, &thr_ptr, task = std::move(this->tasks_base_t::front())]() -> void {
                      task();
                      thr_ptr.reset();
                    }),
                    [](auto &data) -> void { data->join(); });

                // this->workers_base_t::emplace_back(std::move(this->tasks_base_t::front()));
                this->tasks_base_t::pop();

                // static_cast<void>(std::async(std::launch::deferred, [this]() -> void {
                  // if (this->workers_base_t::back().joinable()) {
                    // this->workers_base_t::back().join();
                  // }

                  // {
                    // std::unique_lock<std::mutex> lock(mtx_);
                    // this->workers_base_t::pop_back();
                  // }
                // }));
              }

			  /* Terminate if stopped */
              if (stop_)
                return;
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

      this->tasks_base_t::emplace([task]() -> void { (*task)(); });
    }

    cv_.notify_one();
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
