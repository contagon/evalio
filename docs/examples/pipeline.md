evalio comes with a small number of built-in pipelines, but is made to be easily extensible. Custom pipelines can be created in C++ with nanobind, or in Python. See [evalio-example](https://github.com/contagon/evalio-example) for some examples of building C++ pipelines as well as custom python pipelines.

To get evalio to find your custom pipeline, simply point the environment variable `EVALIO_CUSTOM=my_module` to the module where your pipeline is defined.

To create a pipeline, simply inherit from the `Pipeline` class,

=== "Python"

    ```python
    from evalio.pipelines import Pipeline
    from evalio.types import (
        SE3,
        Point,
        ImuParams,
        LidarParams,
        ImuMeasurement,
        LidarMeasurement,
    )

    class MyPipeline(Pipeline):
        def __init__(self):
            super().__init__()

        # Info
        @staticmethod
        def version() -> str: ...
        @staticmethod
        def url() -> str: ...
        @staticmethod
        def name() -> str: ...
        @staticmethod
        def default_params() -> dict[str, bool | int | float | str]: ...;

        # Getters
        def pose(self) -> SE3: ...
        def map(self) -> list[Point]: ...

        # Setters
        def set_imu_params(self, params: ImuParams): ...
        def set_lidar_params(self, params: LidarParams): ...
        def set_imu_T_lidar(self, T: SE3): ...
        def set_params(self, params: dict[str, bool | int | float | str]): ...

        # Doers
        def initialize(self): ...
        def add_imu(self, mm: ImuMeasurement): ...
        def add_lidar(self, mm: LidarMeasurement) -> list[Point]: ...
    ```

=== "C++"

    ``` c++
    #include "evalio/pipeline.h"
    #include "evalio/types.h"

    #include <nanobind/nanobind.h>
    #include <nanobind/stl/map.h>
    #include <nanobind/stl/string.h>
    #include <nanobind/stl/variant.h>

    namespace nb = nanobind;

    class MyPipeline : public evalio::Pipeline {
    public:
        MyPipeline() : evalio::Pipeline() {}

        // Info
        static std::string version() { ... }
        static std::string url() { ... }
        static std::string name() { ... }
        static std::map<std::string, Param> default_params() { ... };
        
        // Getters
        const SE3 pose() { ... };
        const std::vector<Point> map() { ... };

        // Setters
        void set_imu_params(ImuParams params) { ... };
        void set_lidar_params(LidarParams params) { ... };
        void set_imu_T_lidar(SE3 T) { ... };
        void set_params(std::map<std::string, Param>) { ... };

        // Doers
        void initialize() { ... };
        void add_imu(ImuMeasurement mm) { ... };
        std::vector<Point> add_lidar(LidarMeasurement mm) { ... };
    }
    ```

We'll cover each section of methods in turn.

## Info

The first four methods are all static methods that provide information about the pipeline. `version`, `url`, and `name` are all self-explanatory. `default_params` is a static method that returns a dictionary of the default parameters for the pipeline. This is used to verify parameters before they are passed in, as well as ensure a consistent output for each run.

## Getters

The next two methods are getters for the pose and map. The pose is the most up-to-date estimate for the IMU and is polled after each lidar measurement is passed in. 

The map current map/submap/etc and is only used for visualization purposes.

## Setters

These are to set both dataset specific parameters and pipeline specific parameters. The dataset specific parameters are `imu_params`, `lidar_params`, and `imu_T_lidar`. These are all set before the pipeline is run.

The pipeline specific parameters are set using `set_params`, which takes in a dictionary of parameters. This is used to set any parameters that are specific to the pipeline, such as the number of iterations or the convergence threshold. `default_params` is updated with any parameters and passed at the start of each run.

## Doers
Arguably the most important part. 

`initialize` is called right after all parameters are set. Think of it as a delayed constructor.

`add_imu` is called for each IMU measurement. This is where the IMU data is processed and used to update the pose.

`add_lidar` is called for each lidar measurement. This is where the lidar data is processed and used to update the map. It returns a list of features were extracted from the scan and are used for visualization.

## C++ Building

If done in C++, you will need to build the pipeline as a shared library. This is done by a nanobind wrapper, which can be defined at the bottom of your file as follows,
```c++
NB_MODULE(_core, m) {
  m.doc() = "Custom evalio pipeline example";

  nb::module_ evalio = nb::module_::import_("evalio");

  // Only have to override the static methods here
  // All the others will be automatically inherited from the base class
  nb::class_<MyCppPipeline, evalio::Pipeline>(m, "MyCppPipeline")
      .def(nb::init<>())
      .def_static("name", &MyCppPipeline::name)
      .def_static("url", &MyCppPipeline::url)
      .def_static("default_params", &MyCppPipeline::default_params);
}
```

We recommend then setting everything up to be built with [`scikit-build-core`](https://scikit-build-core.readthedocs.io/en/latest/). You can see [evalio-example](https://github.com/contagon/evalio-example) for an example of how to set this up.

!!! warning

    In order for nanobind to share types between the `evalio` shared object and your custom pipeline, they will have to be compiled with the same version of `libstdc++`. This [pybind PR](https://github.com/pybind/pybind11/pull/5439) discusses this in more detail.

    The "abi_tag" used in your version of evalio can be gotten using `evalio._abi_tag()`, or by running `python -c "import evalio; print(evalio._abi_tag())`". To make sure it matches your nanobind module's, add this to your `NB_MODULE` definition:
    
    ```c++
    m.def("abi_tag", []() { return nb::detail::abi_tag(); });
    ```

That's all there is to it! Pipelines should be fairly easy to implement and are usually just a simple wrapper around your existing code to provide a common interface. Once your pipeline is open-source/published/etc, feel free to make a PR to add it to evalio. This both improves the visibility of your work and of evalio.