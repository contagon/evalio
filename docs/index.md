# evalio

evalio is a tool for **Eval**uating **L**idar-**I**nertial **O**dometry.

Specifically, it provides a common interface for using LIO datasets and LIO pipelines. This allows for easy addition of new datasets and pipelines as well as a common location to evaluate them. This makes benchmarks significantly easier to run and data significantly easier to access. It features,

- Download and manage datasets via the CLI interface
- Simple to use python package API for friction-free access to data
- No ROS dependency! (though it can still load rosbag datasets using the wonderful [rosbags](https://ternaris.gitlab.io/rosbags/) package)
- Easy to add new datasets and pipelines, see the [example](https://github.com/contagon/evalio-example)
- Unified representation of lidar scan, e.g. row (scan-line) major order, stamped at the start of the scan, point stamps are relative from the start of the scan.
- Run pipelines via the CLI interface and yaml config files
- Compute statistics for resulting trajectory runs

Checkout [quickstart](quickstart.md) for a quick overview of the package. Additionally, evalio eases a number of common tasks, check out the examples for some common use cases. 