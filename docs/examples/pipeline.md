evalio comes with a small number of built-in pipelines, but is designed to be extensible.
You can add custom pipelines in either Python or C++ (via nanobind).

For end-to-end examples, see [evalio-example](https://github.com/contagon/evalio-example).

## Loading custom pipelines

evalio discovers custom pipelines from Python modules and auto-registers every `Pipeline`
subclass found in those modules.

- Set `EVALIO_CUSTOM` to one or more comma-separated module names.
- Import `evalio` (or run any `evalio` CLI command).
- evalio imports each module and registers matching pipelines.

```bash
EVALIO_CUSTOM=my_module,another.module evalio ls pipelines
```

You can also register explicitly in Python:

```python
import evalio.pipelines as pl
import my_module

pl.register_pipeline(module=my_module)
```

## Implementing a pipeline

Create a subclass of `Pipeline` and implement the required interface:

=== "Python"

    ```python
    from evalio.pipelines import Pipeline
    from evalio.types import (
        Param,
        SE3,
        Point,
        ImuParams,
        LidarParams,
        ImuMeasurement,
        LidarMeasurement,
        Stamp,
    )


    class MyPipeline(Pipeline):
        @staticmethod
        def name() -> str:
            return "my_pipeline"

        @staticmethod
        def version() -> str:
            return "0.1.0"

        @staticmethod
        def url() -> str:
            return "https://example.com/my_pipeline"

        @staticmethod
        def default_params() -> dict[str, Param]:
            return {
                "max_iters": 8,
                "use_deskew": True,
                "voxel_size": 0.2,
                "mode": "fast",
            }

        def map(self) -> dict[str, list[Point]]:
            return {"map": []}

        def set_imu_params(self, params: ImuParams) -> None:
            ...

        def set_lidar_params(self, params: LidarParams) -> None:
            ...

        def set_imu_T_lidar(self, T: SE3) -> None:
            ...

        def set_params(self, params: dict[str, Param]) -> dict[str, Param]:
            # Return any unused keys from params
            return {}

        def initialize(self) -> None:
            ...

        def add_imu(self, mm: ImuMeasurement) -> None:
            ...

        def add_lidar(self, mm: LidarMeasurement) -> None:
            # Save pose estimate for this stamp
            self.save(mm.stamp, SE3())

            # Optional: save feature groups for visualization
            self.save(mm.stamp, {"corners": [], "planes": []})
    ```

=== "C++"

    ```c++
    #include "evalio/pipeline.h"
    #include "evalio/types.h"

    #include <nanobind/nanobind.h>
    #include <nanobind/stl/map.h>
    #include <nanobind/stl/string.h>
    #include <nanobind/stl/variant.h>

    namespace nb = nanobind;

    class MyPipeline : public evalio::Pipeline {
    public:
        // Info
        static std::string version() { ... }
        static std::string url() { ... }
        static std::string name() { ... }
        static std::map<std::string, evalio::Param> default_params() { ... }

        // Getter
        const evalio::Map<> map() override { ... }

        // Setters
        void set_imu_params(evalio::ImuParams params) override { ... }
        void set_lidar_params(evalio::LidarParams params) override { ... }
        void set_imu_T_lidar(evalio::SE3 T) override { ... }
        std::map<std::string, evalio::Param>
        set_params(std::map<std::string, evalio::Param> params) override {
            return {};
        }

        // Doers
        void initialize() override { ... }
        void add_imu(evalio::ImuMeasurement mm) override { ... }
        void add_lidar(evalio::LidarMeasurement mm) override {
            save(mm.stamp, evalio::SE3{});
            save(mm.stamp, "corners", std::vector<evalio::Point>{});
        }
    };
    ```

## Interface notes

- `name()` and `default_params()` are required and should be stable.
- `version()` and `url()` are optional but strongly recommended.
- `default_params()` drives config parsing and type-checking. Config values must match the
  exact Python/C++ types in these defaults.
- `set_params(...)` should apply known parameters and return any unused ones.
- `add_lidar(...)` does not return features. Use `save(...)` to publish results instead.

## Saving outputs during execution

Use `save(...)` inside your processing methods to emit outputs:

- `save(stamp, pose)` stores an estimated pose.
- `save(stamp, features_map)` stores named feature groups for visualization.
- In C++, `save(stamp, "key", points, "key2", points2, ...)` is also available.
- `map()` returns your current map representation and is used when map visualization is enabled.

Saved values are read by evalio internally through `saved_estimates()`, `saved_features()`,
and `saved_maps()`.

## C++ binding and build

For C++ pipelines, expose your type with nanobind so Python can import it.

```c++
NB_MODULE(_core, m) {
  m.doc() = "Custom evalio pipeline example";

  nb::module_ evalio_mod = nb::module_::import_("evalio");
  auto evalio_pipe = evalio_mod.attr("pipelines").attr("Pipeline");

  // Only have to override the static methods here
  // All the others will be automatically inherited from the base class
  nb::class_<MyCppPipeline, evalio::Pipeline>(m, "MyCppPipeline", evalio_pipe)
      .def(nb::init<>())
      .def_static("name", &MyCppPipeline::name)
      .def_static("version", &MyCppPipeline::version)
      .def_static("url", &MyCppPipeline::url)
      .def_static("default_params", &MyCppPipeline::default_params);
}
```

We recommend then setting everything up to be built with [`scikit-build-core`](https://scikit-build-core.readthedocs.io/en/latest/). You can see [evalio-example](https://github.com/contagon/evalio-example) for an example of how to set this up.

!!! warning

    In order for nanobind to share types between the `evalio` shared object and your custom pipeline, they will have to be compiled with the same version of `libstdc++`. This [pybind PR](https://github.com/pybind/pybind11/pull/5439) discusses this in more detail.

    The "abi_tag" used in your version of evalio can be gotten using `evalio._abi_tag`, or by running `python -c "import evalio; print(evalio._abi_tag)"`. To make sure it matches your nanobind module's, add this to your `NB_MODULE` definition:
     
    ```c++
    m.def("abi_tag", []() { return nb::detail::abi_tag(); });
    ```

Pipelines are typically small wrappers around existing code, exposing a common evalio interface.
Once your pipeline is published, feel free to open a PR to add it to evalio's built-in list.
