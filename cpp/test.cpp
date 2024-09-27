#include "kiss_icp/pipeline/KissICP.hpp"

int main() {
  std::vector<Eigen::Vector4d> points(10);
  for (int i = 0; i < 10; i++) {
    points[i] = Eigen::Vector4d::Random();
  }

  auto config = kiss_icp::pipeline::KISSConfig();
  kiss_icp::pipeline::KissICP icp(config);
  icp.RegisterFrame(points);

  return 1;
}