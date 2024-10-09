#pragma once
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <execution>

#include "types.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace evalio {

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

struct PointCloudMetadata {
  Stamp stamp;
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

// Specialization for Stamp
std::function<void(Stamp&, const uint8_t*)> data_getter(DataType datatype,
                                                        const uint32_t offset) {
  switch (datatype) {
    case UINT16: {
      return [offset](Stamp& value, const uint8_t* data) noexcept {
        value =
            Stamp::from_nsec(*reinterpret_cast<const uint16_t*>(data + offset));
      };
    }
    case UINT32: {
      return [offset](Stamp& value, const uint8_t* data) noexcept {
        value =
            Stamp::from_nsec(*reinterpret_cast<const uint32_t*>(data + offset));
      };
    }
    case FLOAT32: {
      return [offset](Stamp& value, const uint8_t* data) noexcept {
        value = Stamp::from_sec(*reinterpret_cast<const float*>(data + offset));
      };
    }
    case FLOAT64: {
      return [offset](Stamp& value, const uint8_t* data) noexcept {
        value =
            Stamp::from_sec(*reinterpret_cast<const double*>(data + offset));
      };
    }
    default: {
      throw std::runtime_error("Unsupported datatype for stamp");
    }
  }
}

template <typename T>
std::function<void(T&, const uint8_t*)> blank() {
  return [](T&, const uint8_t*) noexcept {};
}

evalio::LidarMeasurement ros_pc2_to_evalio(const PointCloudMetadata& msg,
                                           const std::vector<Field>& fields,
                                           const uint8_t* data) {
  std::function func_x = blank<double>();
  std::function func_y = blank<double>();
  std::function func_z = blank<double>();
  std::function func_intensity = blank<double>();
  std::function func_t = blank<Stamp>();
  std::function func_range = blank<uint32_t>();
  std::function func_row = blank<uint8_t>();
  std::function func_col = blank<uint16_t>();

  if (msg.is_bigendian) {
    throw std::runtime_error("Big endian not supported yet");
  }

  for (const auto& field : fields) {
    if (field.name == "x") {
      func_x = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "y") {
      func_y = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "z") {
      func_z = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "intensity") {
      func_intensity = data_getter<double>(field.datatype, field.offset);
    } else if (field.name == "t" || field.name == "time" ||
               field.name == "stamp" || field.name == "time_offset" ||
               field.name == "timeOffset") {
      func_t = data_getter(field.datatype, field.offset);
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
  mm.points.resize(msg.width * msg.height);

  size_t index = 0;
  for (evalio::Point& point : mm.points) {
    const auto pointStart = data + static_cast<size_t>(index * msg.point_step);
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

// ---------------------- Create python bindings ---------------------- //
void makeConversions(py::module& m) {
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

  py::class_<PointCloudMetadata>(m, "PointCloudMetadata")
      .def(py::init<evalio::Stamp, int, int, int, int, int, int>(),
           py::kw_only(), "stamp"_a, "width"_a, "height"_a, "point_step"_a,
           "row_step"_a, "is_bigendian"_a, "is_dense"_a)
      .def_readwrite("stamp", &PointCloudMetadata::stamp)
      .def_readwrite("width", &PointCloudMetadata::width)
      .def_readwrite("height", &PointCloudMetadata::height)
      .def_readwrite("point_step", &PointCloudMetadata::point_step)
      .def_readwrite("row_step", &PointCloudMetadata::row_step)
      .def_readwrite("is_bigendian", &PointCloudMetadata::is_bigendian)
      .def_readwrite("is_dense", &PointCloudMetadata::is_dense);

  m.def("ros_pc2_to_evalio", [](const PointCloudMetadata& msg,
                                const std::vector<Field>& fields, char* c) {
    return ros_pc2_to_evalio(msg, fields, reinterpret_cast<uint8_t*>(c));
  });
}

}  // namespace evalio