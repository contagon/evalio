// Macro to map a function over a variadic number of values
// https://github.com/swansontec/map-macro
// https://github.com/swansontec/map-macro/issues/11

#define EVAL0(...) __VA_ARGS__
#define EVAL1(...) EVAL0(EVAL0(EVAL0(__VA_ARGS__)))
#define EVAL2(...) EVAL1(EVAL1(EVAL1(__VA_ARGS__)))
#define EVAL3(...) EVAL2(EVAL2(EVAL2(__VA_ARGS__)))
#define EVAL4(...) EVAL3(EVAL3(EVAL3(__VA_ARGS__)))
#define EVAL5(...) EVAL4(EVAL4(EVAL4(__VA_ARGS__)))

#ifdef _MSC_VER
  // MSVC needs more evaluations
  #define EVAL6(...) EVAL5(EVAL5(EVAL5(__VA_ARGS__)))
  #define EVAL(...) EVAL6(EVAL6(__VA_ARGS__))
#else
  #define EVAL(...) EVAL5(__VA_ARGS__)
#endif

#define NOP
#define MAP_TUPLE0(f, x, ...)                                              \
  f(EL0 x, EL1 x, EL2 x, EL3 x) __VA_OPT__(MAP_TUPLE1 NOP(f, __VA_ARGS__))
#define MAP_TUPLE1(f, x, ...)                                              \
  f(EL0 x, EL1 x, EL2 x, EL3 x) __VA_OPT__(MAP_TUPLE0 NOP(f, __VA_ARGS__))
#define MAP_TUPLE(f, ...) __VA_OPT__(EVAL(MAP_TUPLE0(f, __VA_ARGS__)))

#define EL0(a, b, c, d) a
#define EL1(a, b, c, d) b
#define EL2(a, b, c, d) c
#define EL3(a, b, c, d) d
#define STRINGIFY(arg) #arg

// Macro to define default_params and set_params functions for pipeline derivations
// Should help limit boilerplate and mistyping of key names
#define EVALIO_SETUP_PARAMS(...)                                 \
  static std::map<std::string, evalio::Param> default_params() { \
    return {MAP_TUPLE(MAKE_DEFAULT_PARAM, __VA_ARGS__)};         \
  }                                                              \
  std::map<std::string, evalio::Param> set_params(               \
    std::map<std::string, evalio::Param> params                  \
  ) override {                                                   \
    MAP_TUPLE(PARSE_PARAM, __VA_ARGS__)                          \
    return params;                                               \
  }

#define MAKE_DEFAULT_PARAM(type, name, value, save) {STRINGIFY(name), value},

#define PARSE_PARAM(type, name, value, save) \
  {                                          \
    auto i = params.find(STRINGIFY(name));   \
    if (i != params.end()) {                 \
      save = std::get<type>(i->second);      \
      params.erase(i);                       \
    }                                        \
  }
