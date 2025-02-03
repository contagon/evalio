#pragma once
#include <cmath>
#include <cstddef>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <fstream>

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
std::function<void(T &, const uint8_t *)> data_getter(DataType datatype,
                                                      const uint32_t offset) {
  switch (datatype) {
  case UINT8: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const uint8_t *>(data + offset));
    };
  }
  case INT8: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const int8_t *>(data + offset));
    };
  }
  case UINT16: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value =
          static_cast<T>(*reinterpret_cast<const uint16_t *>(data + offset));
    };
  }
  case UINT32: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value =
          static_cast<T>(*reinterpret_cast<const uint32_t *>(data + offset));
    };
  }
  case INT16: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const int16_t *>(data + offset));
    };
  }
  case INT32: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const int32_t *>(data + offset));
    };
  }
  case FLOAT32: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const float *>(data + offset));
    };
  }
  case FLOAT64: {
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const double *>(data + offset));
    };
  }
  default: {
    throw std::runtime_error("Unsupported datatype");
  }
  }
}

// Specialization for Stamp
inline std::function<void(Stamp &, const uint8_t *)>
data_getter(DataType datatype, const uint32_t offset) {
  switch (datatype) {
  case UINT16: {
    return [offset](Stamp &value, const uint8_t *data) noexcept {
      value =
          Stamp::from_nsec(*reinterpret_cast<const uint16_t *>(data + offset));
    };
  }
  case UINT32: {
    return [offset](Stamp &value, const uint8_t *data) noexcept {
      value =
          Stamp::from_nsec(*reinterpret_cast<const uint32_t *>(data + offset));
    };
  }
  case FLOAT32: {
    return [offset](Stamp &value, const uint8_t *data) noexcept {
      value = Stamp::from_sec(*reinterpret_cast<const float *>(data + offset));
    };
  }
  case FLOAT64: {
    return [offset](Stamp &value, const uint8_t *data) noexcept {
      value = Stamp::from_sec(*reinterpret_cast<const double *>(data + offset));
    };
  }
  default: {
    throw std::runtime_error("Unsupported datatype for stamp");
  }
  }
}

template <typename T> std::function<void(T &, const uint8_t *)> blank() {
  return [](T &, const uint8_t *) noexcept {};
}

inline evalio::LidarMeasurement
ros_pc2_to_evalio(const PointCloudMetadata &msg,
                  const std::vector<Field> &fields, const uint8_t *data,
                  const LidarParams &params) {
  std::function func_x = blank<double>();
  std::function func_y = blank<double>();
  std::function func_z = blank<double>();
  std::function func_intensity = blank<double>();
  std::function func_t = blank<Stamp>();
  std::function func_range = blank<uint32_t>();
  std::function func_row = blank<uint8_t>();

  if (msg.is_bigendian) {
    throw std::runtime_error("Big endian not supported yet");
  }

  for (const auto &field : fields) {
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
               field.name == "timeOffset" || field.name == "timestamp") {
      func_t = data_getter(field.datatype, field.offset);
    } else if (field.name == "range") {
      func_range = data_getter<uint32_t>(field.datatype, field.offset);
    } else if (field.name == "row" || field.name == "ring" ||
               field.name == "channel") {
      func_row = data_getter<uint8_t>(field.datatype, field.offset);
    }
  }

  evalio::LidarMeasurement mm(msg.stamp);

  // Check in on some info about the data
  uint8_t first, second;
  func_row(first, data + static_cast<size_t>(0));
  func_row(second, data + static_cast<size_t>(msg.point_step));
  bool row_major = (first == second);
  bool dense_cloud =
      (msg.height * msg.width == params.num_columns * params.num_rows);

  // Figure out how to count the columns
  std::function<void(uint16_t &col, const uint16_t &prev_col,
                     const uint8_t &prev_row, const uint8_t &curr_row)>
      func_col;
  if (row_major) {
    func_col = [](uint16_t &col, const uint16_t &prev_col,
                  const uint8_t &prev_row, const uint8_t &curr_row) {
      if (prev_row != curr_row) {
        col = 0;
      } else {
        col = prev_col + 1;
      }
    };
  } else {
    func_col = [](uint16_t &col, const uint16_t &prev_col,
                  const uint8_t &prev_row, const uint8_t &curr_row) {
      if (curr_row < prev_row) {
        col = prev_col + 1;
      } else {
        col = prev_col;
      }
    };
  }

  mm.points.resize(params.num_columns * params.num_rows);
  uint16_t prev_col = 0;
  uint8_t prev_row = 0;
  // If we got exactly the right number of points in
  if (dense_cloud) {
    // If already row major, fill in the points in place
    if (row_major) {
      size_t index = 0;
      for (evalio::Point &point : mm.points) {
        const auto pointStart =
            data + static_cast<size_t>(index * msg.point_step);
        func_x(point.x, pointStart);
        func_y(point.y, pointStart);
        func_z(point.z, pointStart);
        func_intensity(point.intensity, pointStart);
        func_t(point.t, pointStart);
        func_range(point.range, pointStart);
        func_row(point.row, pointStart);
        func_col(point.col, prev_col, prev_row, point.row);
        prev_col = point.col;
        prev_row = point.row;
        ++index;
      }
      // If not row major, we'll have to organize the points as we go
    } else {
      for (size_t i = 0; i < msg.height * msg.width; i++) {
        const auto pointStart = data + static_cast<size_t>(i * msg.point_step);
        evalio::Point point;
        func_x(point.x, pointStart);
        func_y(point.y, pointStart);
        func_z(point.z, pointStart);
        func_intensity(point.intensity, pointStart);
        func_t(point.t, pointStart);
        func_range(point.range, pointStart);
        func_row(point.row, pointStart);
        func_col(point.col, prev_col, prev_row, point.row);
        prev_col = point.col;
        prev_row = point.row;
        mm.points[point.row * params.num_columns + point.col] = point;
      }
    }
  } else {
    // TODO handle this
    if (row_major) {
      throw std::runtime_error(
          "Non-dense row major point clouds not supported yet");
    } else {
      // fill out row/col for blank points
      for (size_t row = 0; row < params.num_rows; row++) {
        for (size_t col = 0; col < params.num_columns; col++) {
          mm.points[row * params.num_columns + col].row = row;
          mm.points[row * params.num_columns + col].col = col;
        }
      }
      for (size_t i = 0; i < msg.height * msg.width; i++) {
        const auto pointStart = data + static_cast<size_t>(i * msg.point_step);
        evalio::Point point;
        func_x(point.x, pointStart);
        func_y(point.y, pointStart);
        func_z(point.z, pointStart);
        func_intensity(point.intensity, pointStart);
        func_t(point.t, pointStart);
        func_range(point.range, pointStart);
        func_row(point.row, pointStart);
        func_col(point.col, prev_col, prev_row, point.row);
        prev_col = point.col;
        prev_row = point.row;
        mm.points[point.row * params.num_columns + point.col] = point;
      }
    }
  }

  return mm;
}

// The helipr stores them in row major order
// TODO: The helipr format drops points with bad returns - since it's row major,
// its nigh impossible to infer where these were along a scan line
// for now we just tack them on the end
// Largely borrowed from
// https://github.com/minwoo0611/HeLiPR-File-Player/blob/501b338c4be1070fc61a438177c3c0e22b628b30/src/ROSThread.cpp#L444-L454
inline LidarMeasurement helipr_bin_to_evalio(const std::string &filename,
                                             Stamp stamp,
                                             const LidarParams &params) {
  LidarMeasurement mm(stamp);
  mm.points.resize(params.num_columns * params.num_rows);

  std::ifstream file;
  file.open(filename, std::ios::in | std::ios::binary);
  float holder = 0.0;
  uint16_t ring = 0;
  uint32_t nsec = 0;
  uint16_t prev_col = 0;
  uint16_t prev_row = 10; // start with something not 0
  while (!file.eof()) {
    // clang-format off
    Point point;
    file.read(reinterpret_cast<char *>(&holder), sizeof(float)); point.x = holder;
    file.read(reinterpret_cast<char *>(&holder), sizeof(float)); point.y = holder;
    file.read(reinterpret_cast<char *>(&holder), sizeof(float)); point.z = holder;
    file.read(reinterpret_cast<char *>(&holder), sizeof(float)); point.intensity = holder;
    file.read(reinterpret_cast<char *>(&nsec), sizeof(uint32_t)); point.t = Stamp::from_nsec(nsec);
    // file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
    file.ignore(sizeof(uint16_t));
    file.read(reinterpret_cast<char *>(&ring), sizeof(uint16_t)); point.row = ring;
    // file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
    file.ignore(sizeof(uint16_t));
    // clang-format on
    if (prev_row != point.row) {
      point.col = 0;
    } else {
      point.col = prev_col + 1;
    }
    if (point.row >= params.num_rows || point.col >= params.num_columns) {
      std::cout << "HeLiPR point out of bounds\npoint.row: " << +point.row
                << " point.col: " << point.col << std::endl;
      throw -1;
    }

    prev_col = point.col;
    prev_row = point.row;
    mm.points[point.row * params.num_columns + point.col] = point;
  }
  file.close();

  return mm;
}

// ---------------------- Create python bindings ---------------------- //
inline void makeConversions(py::module &m) {
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

  m.def("ros_pc2_to_evalio",
        [](const PointCloudMetadata &msg, const std::vector<Field> &fields,
           char *c, const LidarParams &params) {
          return ros_pc2_to_evalio(msg, fields, reinterpret_cast<uint8_t *>(c),
                                   params);
        });

  m.def("helipr_bin_to_evalio", &helipr_bin_to_evalio);
}

} // namespace evalio