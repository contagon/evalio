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

// ------------------------- Point loading lambdas ------------------------- //
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

inline evalio::LidarMeasurement pc2_to_evalio(const PointCloudMetadata &msg,
                                              const std::vector<Field> &fields,
                                              const uint8_t *data) {
  if (msg.is_bigendian) {
    throw std::runtime_error("Big endian not supported yet");
  }

  evalio::LidarMeasurement mm(msg.stamp);
  mm.points.resize(msg.width * msg.height);

  std::function func_x = blank<double>();
  std::function func_y = blank<double>();
  std::function func_z = blank<double>();
  std::function func_intensity = blank<double>();
  std::function func_t = blank<Stamp>();
  std::function func_range = blank<uint32_t>();
  std::function func_row = blank<uint8_t>();

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

  // Check if stamp is absolute or relative
  Stamp t;
  func_t(t, data);
  std::function<void(Stamp &, const uint8_t *)> func_stamp;
  if (t.sec > 100.0) {
    Stamp scan_stamp = mm.stamp;
    func_stamp = [func_t, scan_stamp](Stamp &stamp,
                                      const uint8_t *data) noexcept {
      func_t(stamp, data);
      stamp = Stamp::from_sec(stamp - scan_stamp);
    };
  } else {
    func_stamp = func_t;
  }

  size_t index = 0;
  for (evalio::Point &point : mm.points) {
    const auto pointStart = data + static_cast<size_t>(index * msg.point_step);
    func_x(point.x, pointStart);
    func_y(point.y, pointStart);
    func_z(point.z, pointStart);
    func_intensity(point.intensity, pointStart);
    func_stamp(point.t, pointStart);
    func_range(point.range, pointStart);
    func_row(point.row, pointStart);
    ++index;
  }

  return mm;
}

// -------------------- Helpers to fill out column index -------------------- //
// Iterates through points to fill in columns
inline void
_fill_col(LidarMeasurement &mm,
          std::function<void(uint16_t &col, const uint16_t &prev_col,
                             const uint8_t &prev_row, const uint8_t &curr_row)>
              func_col) {
  // fill out the first one to kickstart
  uint16_t prev_col = 0;
  uint8_t prev_row = mm.points[0].row;
  for (auto p = mm.points.begin() + 1; p != mm.points.end(); ++p) {
    func_col(p->col, prev_col, prev_row, p->row);
    prev_col = p->col;
    prev_row = p->row;
  }
}

// Fills in column index for row major order
inline void fill_col_row_major(LidarMeasurement &mm) {
  auto func_col = [](uint16_t &col, const uint16_t &prev_col,
                     const uint8_t &prev_row, const uint8_t &curr_row) {
    if (prev_row != curr_row) {
      col = 0;
    } else {
      col = prev_col + 1;
    }
  };

  _fill_col(mm, func_col);
}

// Fills in column index for column major order
inline void fill_col_col_major(LidarMeasurement &mm) {
  auto func_col = [](uint16_t &col, const uint16_t &prev_col,
                     const uint8_t &prev_row, const uint8_t &curr_row) {
    if (curr_row < prev_row) {
      col = prev_col + 1;
    } else {
      col = prev_col;
    }
  };

  _fill_col(mm, func_col);
}

// point cloud loader where rows come in in 0, 8, 1, 9, ... order
inline void fill_col_split_row_velodyne(LidarMeasurement &mm) {
  auto func_row_idx_to_row_seq = [](uint8_t row_idx) {
    if (row_idx < 8) {
      return row_idx * 2;
    } else {
      return (row_idx - 8) * 2 + 1;
    }
  };

  auto func_col = [&func_row_idx_to_row_seq](
                      uint16_t &col, const uint16_t &prev_col,
                      const uint8_t &prev_row, const uint8_t &curr_row) {
    if (func_row_idx_to_row_seq(curr_row) < func_row_idx_to_row_seq(prev_row)) {
      col = prev_col + 1;
    } else {
      col = prev_col;
    }
  };

  _fill_col(mm, func_col);

  // TODO: When I fix the duration stuff, remove this hack
  for (auto &p : mm.points) {
    p.t = Stamp::from_sec(p.t.to_sec() - 5.0 + 0.1);
  }
}

// ------------------------- Helpers for reordering ------------------------- //
inline void reorder_points(LidarMeasurement &mm, size_t num_rows,
                           size_t num_cols) {
  std::vector<Point> points_original = mm.points;
  mm.points = std::vector<Point>(num_rows * num_cols);

  // fill out row/col for blank points
  for (size_t row = 0; row < num_rows; row++) {
    for (size_t col = 0; col < num_cols; col++) {
      mm.points[row * num_cols + col].row = row;
      mm.points[row * num_cols + col].col = col;
    }
  }
  for (auto p : points_original) {
    mm.points[p.row * num_cols + p.col] = p;
  }
}

// ------------------------- Misc other helpers ------------------------- //
// The helipr stores them in row major order
// TODO: The helipr format drops points with bad returns - since it's row
// major, its nigh impossible to infer where these were along a scan line for
// now we just tack them on the end Largely borrowed from
// https://github.com/minwoo0611/HeLiPR-File-Player/blob/501b338c4be1070fc61a438177c3c0e22b628b30/src/ROSThread.cpp#L444-L454
inline LidarMeasurement helipr_bin_to_evalio(const std::string &filename,
                                             Stamp stamp,
                                             const LidarParams &params) {
  LidarMeasurement mm(stamp);
  mm.points.resize(params.num_columns * params.num_rows);
  for (auto &p : mm.points) {
    p = Point();
  }

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
    // if we're off by a byte, just be done early
    if(file.eof()) break;
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

  m.def("pc2_to_evalio", [](const PointCloudMetadata &msg,
                            const std::vector<Field> &fields, char *c) {
    return pc2_to_evalio(msg, fields, reinterpret_cast<uint8_t *>(c));
  });

  m.def("fill_col_row_major", &fill_col_row_major);
  m.def("fill_col_col_major", &fill_col_col_major);
  m.def("reorder_points", &reorder_points);

  // load custom bin format for helipr
  m.def("helipr_bin_to_evalio", &helipr_bin_to_evalio);
  // botanic garden velodyne reordering
  m.def("fill_col_split_row_velodyne", &fill_col_split_row_velodyne);
}

} // namespace evalio