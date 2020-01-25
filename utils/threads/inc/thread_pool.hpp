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

              std::lock_guard<std::mutex> lock(workers_mtx_);
              for (auto it = this->workers_base_t::begin(); it != this->workers_base_t::end(); it++) {
                it->first.join();
              }

              for (auto it = this->workers_base_t::begin(); it != this->workers_base_t::end(); it++) {
                this->workers_base_t::erase(it);
              }

              stop_ = false;
              return;
            } else if (notified_) {

			  std::lock_guard<std::mutex> lock(workers_mtx_);
              std::vector<typename workers_base_t::iterator *> to_erase;
              for (auto it = this->workers_base_t::begin(); it != this->workers_base_t::end(); it++) {
                if (it->second == static_cast<uint32_t>(thread_flag_t::STOPPED)) {
                  it->first.join();
				  to_erase.push_back(&it);
                }
              }

              for (auto it = to_erase.begin(); it != to_erase.end(); it++) {
                typename workers_base_t::iterator *iter = *(it.base());
                this->workers_base_t::erase(*iter);
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

      std::shared_ptr<std::promise<uint32_t *>> flag_sptr(new std::promise<uint32_t *>);
      this->workers_base_t::push_back({std::thread([this, task, flag_sptr](void) -> void {
                                         (*task)();
                                         std::future<uint32_t *> flag_pf = flag_sptr->get_future();
                                         flag_pf.wait();
                                         uint32_t *flag_ptr = flag_pf.get();
                                         *flag_ptr = static_cast<uint32_t>(thread_flag_t::STOPPED);
                                         notified_ = true;
                                         cv_.notify_one();
                                       }),

                                       static_cast<uint32_t>(thread_flag_t::RUNNING)});
      flag_sptr->set_value(&this->workers_base_t::back().second);
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

      std::shared_ptr<std::promise<uint32_t *>> flag_sptr(new std::promise<uint32_t *>);
      this->workers_base_t::push_back({std::thread([this, task, flag_sptr]() -> void {
                                         (*task)();
                                         std::future<uint32_t *> flag_pf = flag_sptr->get_future();
                                         flag_pf.wait();
                                         uint32_t *flag_ptr = flag_pf.get();
                                         *flag_ptr = static_cast<uint32_t>(thread_flag_t::STOPPED);
                                         notified_ = true;
                                         cv_.notify_one();
                                       }),

                                       static_cast<uint32_t>(thread_flag_t::RUNNING)});
      flag_sptr->set_value(&this->workers_base_t::back().second);
    }

    return std::move(res);
  }

private:
  mutable std::mutex mtx_, workers_mtx_;
  mutable std::condition_variable cv_;
  mutable std::atomic<bool> stop_, notified_;
};

#endif /* THREAD_POOL_HPP */
