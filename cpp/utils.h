#pragma once
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <execution>
// #include <functional>
// #include <numeric>
// #include <type_traits>

#include "types.h"

namespace py = pybind11;
using namespace pybind11::literals;

enum DataType {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
};

struct Field {
  std::string name;
  DataType datatype;
  uint32_t offset;
};

// TODO: is this necessary? Or should we just send over all the data into the
// function
struct PointCloud2 {
  std::vector<Field> fields;
  std::vector<uint8_t> data;
  evalio::Stamp stamp;
  int width;
  int height;
  int point_step;
  int row_step;
  int is_bigendian;
  int is_dense;
};

template <typename T>
std::function<void(T&, const uint8_t*)> data_getter(DataType datatype,
                                                    const uint32_t offset) {
  switch (datatype) {
    case UINT8: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value =
            static_cast<T>(*reinterpret_cast<const uint8_t*>(data + offset));
      };
    }
    case INT8: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value = static_cast<T>(*reinterpret_cast<const int8_t*>(data + offset));
      };
    }
    case UINT16: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value =
            static_cast<T>(*reinterpret_cast<const uint16_t*>(data + offset));
      };
    }
    case UINT32: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value =
            static_cast<T>(*reinterpret_cast<const uint32_t*>(data + offset));
      };
    }
    case INT16: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value =
            static_cast<T>(*reinterpret_cast<const int16_t*>(data + offset));
      };
    }
    case INT32: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value =
            static_cast<T>(*reinterpret_cast<const int32_t*>(data + offset));
      };
    }
    case FLOAT32: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value = static_cast<T>(*reinterpret_cast<const float*>(data + offset));
      };
    }
    case FLOAT64: {
      return [offset](T& value, const uint8_t* data) noexcept {
        value = static_cast<T>(*reinterpret_cast<const double*>(data + offset));
      };
    }
    default: {
      throw std::runtime_error("Unsupported datatype");
    }
  }
}

template <typename T>
std::function<void(T&, const uint8_t*)> blank() {
  return [](T&, const uint8_t*) noexcept {};
}

evalio::LidarMeasurement pointcloud2_to_evalio(const PointCloud2& msg) {
  std::function func_x = blank<double>();
  std::function func_y = blank<double>();
  std::function func_z = blank<double>();
  std::function func_intensity = blank<double>();
  std::function func_t = blank<uint32_t>();
  std::function func_range = blank<uint32_t>();
  std::function func_row = blank<uint8_t>();
  std::function func_col = blank<uint16_t>();

  if (msg.is_bigendian) {
    throw std::runtime_error("Big endian not supported yet");
  }

  for (const auto& field : msg.fields) {
    if (field.name == "x") {
      func_x = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "y") {
      func_y = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "z") {
      func_z = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "intensity") {
      func_intensity = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "t") {
      func_t = data_getter<uint32_t>(field.datatype, field.offset);
    } else if (field.name == "range") {
      func_range = data_getter<uint32_t>(field.datatype, field.offset);
    } else if (field.name == "row") {
      func_row = data_getter<uint8_t>(field.datatype, field.offset);
    } else if (field.name == "col") {
      // TODO: Custom column function?
      func_col = data_getter<uint16_t>(field.datatype, field.offset);
    }
  }

  evalio::LidarMeasurement mm(msg.stamp);
  mm.points.resize(msg.data.size() / msg.point_step);

  // TODO: Need to consider row_step at all?
  size_t index = 0;
  for (evalio::Point& point : mm.points) {
    const auto pointStart =
        msg.data.data() + static_cast<size_t>(index * msg.point_step);
    func_x(point.x, pointStart);
    func_y(point.y, pointStart);
    func_z(point.z, pointStart);
    func_intensity(point.intensity, pointStart);
    func_t(point.t, pointStart);
    func_range(point.range, pointStart);
    func_row(point.row, pointStart);
    func_col(point.col, pointStart);
    ++index;
  }

  return mm;
}

// ------------------------- Create python bindings -------------------------
// //
void makeUtils(py::module& m) {
  py::enum_<DataType>(m, "DataType")
      .value("UINT8", DataType::UINT8)
      .value("INT8", DataType::INT8)
      .value("UINT16", DataType::UINT16)
      .value("UINT32", DataType::UINT32)
      .value("INT16", DataType::INT16)
      .value("INT32", DataType::INT32)
      .value("FLOAT32", DataType::FLOAT32)
      .value("FLOAT64", DataType::FLOAT64);

  py::class_<Field>(m, "Field")
      .def(py::init<std::string, DataType, uint32_t>(), py::kw_only(), "name"_a,
           "datatype"_a, "offset"_a)
      .def_readwrite("name", &Field::name)
      .def_readwrite("datatype", &Field::datatype)
      .def_readwrite("offset", &Field::offset);

  py::class_<PointCloud2>(m, "PointCloud2")
      .def(py::init<std::vector<Field>, std::vector<uint8_t>, evalio::Stamp,
                    int, int, int, int, int, int>(),
           py::kw_only(), "fields"_a, "data"_a, "stamp"_a, "width"_a,
           "height"_a, "point_step"_a, "row_step"_a, "is_bigendian"_a,
           "is_dense"_a)
      .def_readwrite("fields", &PointCloud2::fields)
      .def_readwrite("data", &PointCloud2::data)
      .def_readwrite("stamp", &PointCloud2::stamp)
      .def_readwrite("width", &PointCloud2::width)
      .def_readwrite("height", &PointCloud2::height)
      .def_readwrite("point_step", &PointCloud2::point_step)
      .def_readwrite("row_step", &PointCloud2::row_step)
      .def_readwrite("is_bigendian", &PointCloud2::is_bigendian)
      .def_readwrite("is_dense", &PointCloud2::is_dense);

  m.def("pointcloud2_to_evalio", &pointcloud2_to_evalio);
}