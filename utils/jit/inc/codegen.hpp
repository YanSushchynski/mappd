#ifndef CODEGEN_HPP
#define CODEGEN_HPP

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fmt/format.h>
#include <libconfig.h++>

#include "env_utils.hpp"
#include "hash.hpp"
#include "tuple.hpp"
#include "type_traits_ex.hpp"

struct codegen_t {

  template <uint64_t sum> struct _ {
    static_assert(sum == sum64("") || sum == sum64("d") || sum == sum64("p") || sum == sum64("s") ||
                      sum == sum64("dp") || sum == sum64("pd") || sum == sum64("ds") || sum == sum64("sd") ||
                      sum == sum64("sp") || sum == sum64("ps") || sum == sum64("dsp") || sum == sum64("dps") ||
                      sum == sum64("sdp") || sum == sum64("spd") || sum == sum64("pds") || sum == sum64("psd"),
                  "Use only 's', 'd', 'p' chars to enable preffix, suffix or delimeter");

    static constexpr const bool only_delimiter = sum == sum64("d");
    static constexpr const bool only_preffix = sum == sum64("p");
    static constexpr const bool only_preffix_with_delim = sum == sum64("pd");
    static constexpr const bool only_suffix = sum == sum64("s");
    static constexpr const bool only_suffix_with_delim = sum == sum64("sd");
    static constexpr const bool with_suffix_and_preffix = sum == sum64("sp");
    static constexpr const bool with_all = sum == sum64("spd");
    static constexpr const bool without_any = sum == 0u;
  };

  static std::string gen_port_src_code(libconfig::Setting &cfg);
  static std::string gen_component_src_code(libconfig::Setting &cfg);
  static std::string gen_composition_src_code(libconfig::Setting &cfg);
  static std::string gen_env_src_code(libconfig::Setting &cfg);

  template <uint32_t N, typename DataType>
  static std::string enumerate_(const DataType (&args)[N], const std::string &preffix = "",
                                const std::string &suffix = "", const std::string &delimiter = ",") {
    std::string fmt = "";
    auto tp = std::array_to_tuple(args);
    for (unsigned int i = 0u; i < N; i++)
      (i == N - 1u) ? fmt += (preffix == "" && suffix != "")
                                 ? fmt::format("{{{0}}} {1}", i, suffix)
                                 : (suffix == "" && preffix != "")
                                       ? fmt::format("{0} {{{1}}}", preffix, i)
                                       : (preffix == "" && suffix == "")
                                             ? fmt::format("{{{0}}}", i)
                                             : fmt::format("{0} {{{1}}} {2}", preffix, i, suffix)

                    : fmt += (preffix == "" && suffix != "")
                                 ? fmt::format("{{{0}}} {1}{2} ", i, suffix, delimiter)
                                 : (suffix == "" && preffix != "")
                                       ? fmt::format("{0} {{{1}}}{2} ", preffix, i, delimiter)
                                       : (preffix == "" && suffix == "")
                                             ? fmt::format("{{{0}}}{1} ", i, delimiter)
                                             : fmt::format("{0} {{{1}}} {2}{3} ", preffix, i, suffix, delimiter);

    auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
    return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), tp));
  }

  template <typename... Args>
  static std::string enumerate_(const std::tuple<Args...> &args, const std::string &preffix = "",
                                const std::string &suffix = "", const std::string &delimiter = ",") {
    std::string fmt = "";
    for (unsigned int i = 0u; i < sizeof...(Args); i++)
      (i == sizeof...(Args) - 1u)
          ? fmt += (preffix == "" && suffix != "")
                       ? fmt::format("{{{0}}} {1}", i, suffix)
                       : (suffix == "" && preffix != "")
                             ? fmt::format("{0} {{{1}}}", preffix, i)
                             : (preffix == "" && suffix == "") ? fmt::format("{{{0}}}", i)
                                                               : fmt::format("{0} {{{1}}} {2}", preffix, i, suffix)

          : fmt += (preffix == "" && suffix != "")
                       ? fmt::format("{{{0}}} {1}{2} ", i, suffix, delimiter)
                       : (suffix == "" && preffix != "")
                             ? fmt::format("{0} {{{1}}}{2} ", preffix, i, delimiter)
                             : (preffix == "" && suffix == "")
                                   ? fmt::format("{{{0}}}{1} ", i, delimiter)
                                   : fmt::format("{0} {{{1}}} {2}{3} ", preffix, i, suffix, delimiter);

    auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
    return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), args));
  }

  template <typename DataType>
  static std::string enumerate_(const std::vector<DataType> &args, const std::string &preffix = "",
                                const std::string &suffix = "", const std::string &delimiter = ",") {
    std::string res = "";
    for (unsigned int i = 0u; i < args.size(); i++)
      (i == args.size() - 1u)
          ? res += (preffix == "" && suffix != "")
                       ? fmt::format("{0} {1}", args[i], suffix)
                       : (suffix == "" && preffix != "")
                             ? fmt::format("{0} {1}", preffix, args[i])
                             : (preffix == "" && suffix == "") ? fmt::format("{0}", args[i])
                                                               : fmt::format("{0} {1} {2}", preffix, args[i], suffix)

          : res += (preffix == "" && suffix != "")
                       ? fmt::format("{0} {1}{2} ", args[i], suffix, delimiter)
                       : (suffix == "" && preffix != "")
                             ? fmt::format("{0} {1}{2} ", preffix, args[i], delimiter)
                             : (preffix == "" && suffix == "")
                                   ? fmt::format("{0}{1} ", args[i], delimiter)
                                   : fmt::format("{0} {1} {2}{3} ", preffix, args[i], suffix, delimiter);

    return std::move(res);
  }

  template <uint64_t sum, typename... Args> static std::string enumerate_(_<sum> parameter, const Args &... args) {

    std::tuple<const Args &...> tp(args...);
    std::string fmt = "";
    if constexpr (parameter.with_all) {

      static constexpr int preffix_index = sizeof...(Args) - 3u;
      static constexpr int suffix_index = sizeof...(Args) - 2u;
      static constexpr int delim_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 3u;
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u)
            ? fmt += fmt::format("{0} {{{1}}} {2}", std::get<preffix_index>(tp), i, std::get<suffix_index>(tp))
            : fmt += fmt::format("{0} {{{1}}} {2}{3} ", std::get<preffix_index>(tp), i, std::get<suffix_index>(tp),
                                 std::get<delim_index>(tp));

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else if constexpr (parameter.with_suffix_and_preffix) {

      static constexpr int preffix_index = sizeof...(Args) - 2u;
      static constexpr int suffix_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 2u;
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u)
            ? fmt += fmt::format("{0} {{{1}}} {2}", std::get<preffix_index>(tp), i, std::get<suffix_index>(tp))
            : fmt += fmt::format("{0} {{{1}}} {2}, ", std::get<preffix_index>(tp), i, std::get<suffix_index>(tp));

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else if constexpr (parameter.only_preffix) {

      static constexpr int preffix_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 1u;
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u) ? fmt += fmt::format("{0} {{{1}}}", std::get<preffix_index>(tp), i)
                                 : fmt += fmt::format("{0} {{{1}}}, ", std::get<preffix_index>(tp), i);

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else if constexpr (parameter.only_preffix_with_delim) {

      static constexpr int preffix_index = sizeof...(Args) - 2u;
      static constexpr int delim_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 2u;
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u)
            ? fmt += fmt::format("{0} {{{1}}}", std::get<preffix_index>(tp), i)
            : fmt += fmt::format("{0} {{{1}}}{2} ", std::get<preffix_index>(tp), i, std::get<delim_index>(tp));

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else if constexpr (parameter.only_suffix) {

      static constexpr int suffix_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 1u;
      auto payload = std::tuple_slice<0, payload_size>(tp);
      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u) ? fmt += fmt::format("{{{0}}} {1}", i, std::get<suffix_index>(tp))
                                 : fmt += fmt::format("{{{0}}} {1}, ", i, std::get<suffix_index>(tp));

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else if constexpr (parameter.only_suffix_with_delim) {

      static constexpr int suffix_index = sizeof...(Args) - 2u;
      static constexpr int delim_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 2u;
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u)
            ? fmt += fmt::format("{{{0}}} {1}", i, std::get<suffix_index>(tp))
            : fmt += fmt::format("{{{0}}} {1}{2} ", i, std::get<suffix_index>(tp), std::get<delim_index>(tp));

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else if constexpr (parameter.only_delimiter) {

      static constexpr int delim_index = sizeof...(Args) - 1u;
      static constexpr int payload_size = sizeof...(Args) - 1u;
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u) ? fmt += fmt::format("{{{0}}}", i)
                                 : fmt += fmt::format("{{{0}}}{1} ", i, std::get<delim_index>(tp));

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));

    } else {

      static constexpr int payload_size = sizeof...(Args);
      auto payload = std::tuple_slice<0, payload_size>(tp);

      for (unsigned int i = 0u; i < payload_size; i++)
        (i == payload_size - 1u) ? fmt += fmt::format("{{{0}}}", i) : fmt += fmt::format("{{{0}}}, ", i);

      auto format_wrapper = [](auto fmt, const auto &... args) -> auto { return fmt::format(fmt, args...); };
      return std::apply(format_wrapper, std::tuple_cat(std::make_tuple(fmt), payload));
    }
  }

  template <typename... Args> static std::string wrap_by_(const char (&style)[4u], const Args &... args) {
    return style[0u] + enumerate_(_<"d"_s64>(), args..., std::string(1u, style[1u])) + style[2u];
  };

  template <uint32_t N, typename DataType>
  static std::string wrap_by_(const char (&style)[4u], const DataType (&args)[N]) {
    return style[0u] + enumerate_(args, "", "", std::string(1u, style[1u])) + style[2u];
  }

  template <typename... Args> static std::string wrap_by_(const char (&style)[4u], const std::tuple<Args...> &args) {
    return style[0u] + enumerate_(args, "", "", std::string(1u, style[1u])) + style[2u];
  }

  template <typename DataType> static std::string wrap_by_(const char (&style)[4u], const std::vector<DataType> &args) {
    return style[0u] + enumerate_(args, "", "", std::string(1u, style[1u])) + style[2u];
  }

  template <typename... Args> static std::string wrap_by_(const char (&style)[3u], const Args &... args) {
    return style[0u] + enumerate_(_<"d"_s64>(), args..., "") + style[1u];
  };

  template <uint32_t N, typename DataType>
  static std::string wrap_by_(const char (&style)[3u], const DataType (&args)[N]) {
    return style[0u] + enumerate_(args, "", "", "") + style[1u];
  }

  template <typename... Args> static std::string wrap_by_(const char (&style)[3u], const std::tuple<Args...> &args) {
    return style[0u] + enumerate_(args, "", "", "") + style[1u];
  }

  template <typename DataType> static std::string wrap_by_(const char (&style)[3u], const std::vector<DataType> &args) {
    return style[0u] + enumerate_(args, "", "", "") + style[1u];
  }

  template <template <typename...> class Params, typename... _Params, template <typename...> class Args,
            typename... _Args>
  static std::string template_def_func_style_(const std::string &type, const Params<_Params...> &params,
                                   const std::string &varname, const Args<_Args...> &args) {
    return type + wrap_by_("<,>", params) + " " + varname + wrap_by_("(,)", args) + ";";
  }

  template <template <typename...> class Params, typename... _Params, template <typename...> class Args,
            typename... _Args>
  static std::string template_def_array_style_(const std::string &type, const Params<_Params...> &params,
                                               const std::string &varname, const Args<_Args...> &args) {
    return type + wrap_by_("<,>", params) + " " + varname + wrap_by_("{,}", args) + ";";
  }

  template <template <typename...> class Params, typename... _Params, template <typename...> class Args,
            typename... _Args>
  static std::string template_def_func_style_(const std::string &type, const Params<_Params...> &params,
                                              const Args<_Args...> &args) {
    return type + wrap_by_("<,>", params) + wrap_by_("(,)", args) + ";";
  }

  template <template <typename...> class Params, typename... _Params, template <typename...> class Args,
            typename... _Args>
  static std::string template_def_array_style_(const std::string &type, const Params<_Params...> &params,
                                               const Args<_Args...> &args) {
    return type + wrap_by_("<,>", params) + wrap_by_("{,}", args) + ";";
  }

  template <template <typename...> class Params, typename... _Params>
  static std::string template_decl_func_style_(const std::string &type, const Params<_Params...> &params) {
    return type + wrap_by_("<,>", params);
  }

  template <template <typename...> class Params, typename... _Params>
  static std::string template_decl_array_style_(const std::string &type, const Params<_Params...> &params) {
    return type + wrap_by_("<,>", params);
  }

  template <template <typename...> class Args, typename... _Args>
  static std::string call_(const std::string &name, const Args<_Args...> &args) {
    return name + wrap_by_("(,)", args) + ";";
  }

  template <template <typename...> class Qualifiers, typename... _Qualifiers, typename DataType>
  static std::string var_def_(const DataType &initval, const std::string &name,
                              const Qualifiers<_Qualifiers...> &qualifiers,
                              const std::string &type_name = typestr<DataType>) {
    return fmt::format(type_name + " " + enumerate_(qualifiers, "", "", " ") + " " + name + " = " + "{};", initval);
  }

  template <uint32_t N, typename DataType>
  static std::string var_def_(const DataType &initval, const std::string &name, const std::string (&qualifiers)[N],
                              const std::string &type_name = typestr<DataType>) {
    return fmt::format(type_name + " " + enumerate_(qualifiers, "", "", " ") + " " + name + " = " + "{};", initval);
  }

  template <template <typename...> class Qualifiers, typename... _Qualifiers>
  static std::string var_def_(const std::string &type_name, const std::string &name,
                              const Qualifiers<_Qualifiers...> &qualifiers) {
    return type_name + " " + enumerate_(qualifiers, "", "", " ") + " " + name + ";";
  }

  template <uint32_t N>
  static std::string var_def_(const std::string &type_name, const std::string &name,
                              const std::string (&qualifiers)[N]) {
    return type_name + " " + enumerate_(qualifiers, "", "", " ") + " " + name + ";";
  }

  template <template <typename...> class Qualifiers, typename... _Qualifiers, typename DataType>
  static std::string var_def_(const DataType &initval, const std::string &name,
                              const std::string &type_name = typestr<DataType>) {
    return fmt::format(type_name + " " + name + " = " + "{};", initval);
  }

  template <typename DataType>
  static std::string var_def_(const DataType &initval, const std::string &name,
                              const std::string &type_name = typestr<DataType>) {
    return fmt::format(type_name + " " + name + " = " + "{};", initval);
  }

  static std::string var_def_(const std::string &initval, const std::string &name,
                              const std::string &type_name = "auto") {
    return fmt::format(type_name + " " + name + " = " + "{};", initval);
  }

  template <template <typename...> class Fields, typename... _Fields>
  static std::string struct_decl_(const std::string &struct_name, const Fields<_Fields...> &fields) {
    return "struct " + struct_name + wrap_by_("{}", fields) + ";";
  }

  template <uint32_t N>
  static std::string struct_decl_(const std::string &struct_name, const std::string (&fields)[N]) {
    return "struct " + struct_name + wrap_by_("{}", fields) + ";";
  }
};

#endif /* CODEGEN_HPP */
