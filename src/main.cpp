/*clang++ -g -std=gnu++17 -o main.elf main.cpp ../utils/jit/src/jit.cpp ../utils/env/src/env_utils.cpp -I ../env/inc/ -I
 * ../utils/array/inc/ -I ../utils/tuple/inc/ -I ../utils/type/inc/ -I ../utils/function/inc/ -I ../utils/env/inc/ -I
 * ../utils/jit/inc/ -I ../inc/ -ldl -lconfig++ && ./main.elf -c ./config.conf*/

#include "dl.hpp"
#include "env_dynamic.hpp"
#include "env_static.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

#include <dirent.h>
#include <libconfig.h++>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

struct opts_t {
  std::string config_file_path = "";
};

struct root_tokens_t {
  static constexpr std::string_view version = "version";
  static constexpr std::string_view rootdir = "rootdir";
  static constexpr std::string_view envs = "environments";
};

struct env_tokens_t {
  static constexpr std::string_view name = "name";
  static constexpr std::string_view cl_args = "cmdline_args";
  static constexpr std::string_view headers_path = "headers_path";
  static constexpr std::string_view components_path = "components_path";
  static constexpr std::string_view mountpoint = "mountpoint";
  static constexpr std::string_view signals = "signals";
};

struct signal_tokens_t {
  static constexpr std::string_view name = "name";
  static constexpr std::string_view priority = "priority";
  static constexpr std::string_view interval_us = "interval_us";
};

static opts_t opts;
static std::string progname;

static std::pair<int, char **> str_to_c_list(std::string &args) {
  char *str = const_cast<char *>(args.c_str());
  int init_size = std::strlen(str);
  char *ptr = std::strtok(str, " ");
  int argc = 0;
  char **argv = nullptr;

  while (ptr != nullptr) {
    argc++;
    argv = static_cast<char **>(std::realloc(argv, argc * sizeof(char *)));
    argv[argc - 1u] = static_cast<char *>(std::malloc(std::strlen(ptr) + 1u));
    std::strncpy((argv)[argc - 1u], ptr, std::strlen(ptr) + 1u);
    ptr = strtok(nullptr, " ");
  }

  return {argc, argv};
}

static void usage(void) {
  std::printf("Usage : %s [opts]\r\n", progname.c_str());
  std::terminate();
}

static void start(const char *file) {
  libconfig::Config cfg;

  try {
    cfg.readFile(file);
  } catch (const libconfig::FileIOException &fioex) {

    std::printf("Error : %s\r\n", fioex.what());
    std::terminate();
  } catch (const libconfig::ParseException &pex) {

    std::printf("Error : %s at %s:%i\r\n", pex.getError(), pex.getFile(), pex.getLine());
    std::terminate();
  }

  try {

    libconfig::Setting &root = cfg.getRoot();
    std::string rootdir = root.lookup(root_tokens_t::rootdir.data());
    std::string version = root.lookup(root_tokens_t::version.data());
    libconfig::Setting &envs_cfg = root.lookup(root_tokens_t::envs.data());
    std::map<const std::string, dlmodule> envs;
    std::vector<pid_t> forked_pids;

    for (libconfig::Setting &env_cfg : envs_cfg) {

      std::function<void(const std::string &, const std::string &)> process_dir =
          [&rootdir](const std::string &env_root, const std::string &dir_name) -> void {
        const char *rootdir_abs_path = realpath(rootdir.c_str(), nullptr);

        if (rootdir_abs_path) {

          std::string dir_abs_path = std::string(rootdir_abs_path) + "/" + dir_name;
          DIR *dir = opendir(dir_abs_path.c_str());

          if (dir)
            closedir(dir);
          else
            mkdir(dir_abs_path.c_str(), 0565);
        } else {

          mkdir(rootdir_abs_path, 0565);
        }
      };

      std::string env_name = env_cfg.lookup(env_tokens_t::name.data());
      process_dir(rootdir + "/" + env_name, env_cfg.lookup(env_tokens_t::headers_path.data()));
      process_dir(rootdir + "/" + env_name, env_cfg.lookup(env_tokens_t::components_path.data()));
      process_dir(rootdir + "/" + env_name, env_cfg.lookup(env_tokens_t::mountpoint.data()));

      /* Create module */
      // envs.insert(std::make_pair(env_name, dlmodule(env_cfg)));

      /* Check module */
      // dlmodule *p_env_mod = &envs.at(env_name);
      // std::printf("%p, %s\r\n%p, %s\r\n", p_env_mod->info()->dli_fbase, p_env_mod->info()->dli_fname,
      // p_env_mod->info()->dli_saddr, p_env_mod->info()->dli_sname);

      std::string env_cl_args = env_cfg.lookup(env_tokens_t::cl_args.data());

      /*Create cmdline args for env*/
      auto [env_cl_argc, env_cl_argv] = str_to_c_list(env_cl_args);

      /* Create another process for run environment */
      int pid = fork();

      if (!pid) {

        /* Forked - configure and run env*/
        // env_base_t *p_env = p_env_mod->sym<env_base_t>(env_name);
        // p_env->configure(env_cfg);
        // p_env->run(env_cl_argc, env_cl_argv);
        std::exit(EXIT_SUCCESS);
      } else {

        /* Parent - create next env */
        forked_pids.push_back(pid);
        continue;
      }
    }

    for (pid_t &pid : forked_pids) {

      int status;
      waitpid(pid, &status, 0);
    }

    std::exit(EXIT_SUCCESS);
  } catch (const libconfig::SettingNotFoundException &snfex) {

    std::printf("Error : %s at %s\r\n", snfex.what(), snfex.getPath());
    std::terminate();
  } catch (const libconfig::SettingTypeException &stex) {

    std::printf("Error : %s at %s\r\n", stex.what(), stex.getPath());
    std::terminate();
  } catch (const std::runtime_error &e) {

    std::printf("Error : %s\r\n", e.what());
    std::terminate();
  }
}

int __attribute__((noreturn)) main(int argc, char *argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  static const char *opt_str = "c:e:h?";
  progname = argv[0];
  std::string default_conf_path = "/etc/" + progname + "/" + progname + ".conf";
  const char *real_default_conf_path = realpath(("/etc/" + progname + "/" + progname + ".conf").c_str(), nullptr);
  if (real_default_conf_path)
    opts.config_file_path = real_default_conf_path;

  int opt = getopt(argc, argv, opt_str);

  while (opt != -1) {
    switch (opt) {

    case 'c': {
      const char *path = realpath(optarg, nullptr);
      if (path)
        opts.config_file_path = path;
    } break;

    case 'h':
    case '?':
      usage();
      break;

    default:
      break;
    }

    opt = getopt(argc, argv, opt_str);
  }

  bool file_exists = !opts.config_file_path.empty();

  if (!file_exists)
    throw std::runtime_error("Config file access error");

  start(opts.config_file_path.c_str());
}
