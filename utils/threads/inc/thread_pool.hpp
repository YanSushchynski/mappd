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

  enum struct event_t : uint32_t { UNDEFINED = 0u, ADDED, PERFORMED };

public:
  explicit thread_pool()
      : stop_(false), notified_(false), event_(event_t::UNDEFINED), event_related_thread_(nullptr),
        observer_base_t(std::thread([this]() -> auto {
          while (true) {
            {
              std::unique_lock<std::mutex> lock(mtx_);
              cv_.wait(lock, [this]() -> bool { return stop_ || notified_; });
            }

            if (stop_) {
              return;
            } else if (notified_) {

              switch (event_) {
              case event_t::ADDED: {
                std::shared_ptr<std::thread> thr_ptr(nullptr);
                this->workers_base_t::emplace_back(
                    [this, &thr_ptr, task = std::move(this->tasks_base_t::front())]() -> void {
                      task();
                      event_ = event_t::PERFORMED;
                      event_related_thread_ = thr_ptr.get();
                      notified_ = true;
                      cv_.notify_one();
                    });

                std::shared_ptr<std::thread> p(&this->workers_base_t::back(), [](auto &thread) -> void {});
                thr_ptr.swap(p);
                this->tasks_base_t::pop();
              } break;

              case event_t::PERFORMED: {
                if (event_related_thread_->joinable())
                  event_related_thread_->join();

                for (auto it = this->workers_base_t::begin(); it != this->workers_base_t::end(); it++) {
                  if (event_related_thread_ == it.base()) {
                    this->workers_base_t::erase(it);
                  }
                }
              } break;

              case event_t::UNDEFINED:
              default:
                break;
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

    notified_ = true;
    event_ = event_t::ADDED;
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

    notified_ = true;
    event_ = event_t::ADDED;
    cv_.notify_one();
    return std::move(res);
  }

private:
  mutable std::mutex mtx_;
  mutable std::condition_variable cv_;
  mutable std::atomic<bool> stop_, notified_;
  mutable std::atomic<event_t> event_;
  mutable std::thread *event_related_thread_;
};

#endif /* THREAD_POOL_HPP */
