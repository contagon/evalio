from . import Pipeline
from evalio.cli import PipelineBuilder
from evalio.types import (
    SE3,
    LidarParams,
    LidarMeasurement,
    ImuMeasurement,
    ImuParams,
    Point,
)
from typing import Optional

from copy import deepcopy


class IntensityCleaner(Pipeline):
    def __init__(self) -> None:
        super().__init__()
        self._pipeline: Optional[Pipeline] = None
        self.clean = True

    @property
    def pipeline(self) -> Pipeline:
        if self._pipeline is None:
            raise Exception("Pipeline not set")
        else:
            return self._pipeline

    # Info
    @staticmethod
    def name() -> str:
        return "cleaner"

    @staticmethod
    def url() -> str:
        return "here"

    @staticmethod
    def default_params() -> dict[str, bool | int | float | str]:
        return {"wrap": "kiss", "clean": True}

    # Getters
    def pose(self) -> SE3:
        return self.pipeline.pose()

    def map(self) -> list[Point]:
        return self.pipeline.map()

    # Setters
    def set_params(self, params: dict[str, bool | int | float | str]) -> None:
        params = deepcopy(params)
        if "wrap" in params:
            self._pipeline = PipelineBuilder._get_pipeline(params.pop("wrap"))()
        else:
            self._pipeline = PipelineBuilder._get_pipeline("kiss")()

        if "clean" in params:
            clean = params.pop("clean")
            if isinstance(clean, bool):
                self.clean = clean
            else:
                raise Exception("clean must be a boolean")

        self.pipeline.set_params(params)

    def set_imu_params(self, params: ImuParams):
        self.pipeline.set_imu_params(params)

    def set_lidar_params(self, params: LidarParams):
        self.pipeline.set_lidar_params(params)

    def set_imu_T_lidar(self, T_imu_lidar: SE3):
        self.pipeline.set_imu_T_lidar(T_imu_lidar)

    # Doers
    def initialize(self) -> None:
        self.pipeline.initialize()

    def add_imu(self, mm: ImuMeasurement) -> None:
        self.pipeline.add_imu(mm)

    def add_lidar(
        self, mm: LidarMeasurement, init: Optional[SE3] = None
    ) -> list[Point]:
        if self.clean:
            # Threshold intensity to [0,1]
            # maximum = max(p.intensity for p in mm.points)
            # print(maximum)
            for p in mm.points:
                p.intensity *= 10.0
        return self.pipeline.add_lidar(mm, init)
