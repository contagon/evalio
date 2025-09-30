"""
These are extended types that do depend on other parts of evalio.
"""

from __future__ import annotations

from enum import Enum
from dataclasses import dataclass
from typing import Any, Optional, Self
from evalio.types.base import Param, Metadata

from evalio import pipelines as pl, datasets as ds
from evalio.utils import print_warning


class ExperimentStatus(Enum):
    Complete = "complete"
    Fail = "fail"
    Started = "started"


@dataclass(kw_only=True)
class Experiment(Metadata):
    name: str
    """Name of the experiment."""
    sequence: str
    """Dataset used to run the experiment."""
    sequence_length: int
    """Length of the sequence, if set"""
    pipeline: str
    """Pipeline used to generate the trajectory."""
    pipeline_version: str
    """Version of the pipeline used."""
    pipeline_params: dict[str, Param]
    """Parameters used for the pipeline."""
    status: ExperimentStatus = ExperimentStatus.Started
    """Status of the experiment, e.g. "success", "failure", etc."""
    total_elapsed: Optional[float] = None
    """Total time taken for the experiment, as a string."""
    max_step_elapsed: Optional[float] = None
    """Maximum time taken for a single step in the experiment, as a string."""

    def to_dict(self) -> dict[str, Any]:
        d = super().to_dict()
        d["status"] = self.status.value
        return d

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        if "status" in data:
            data["status"] = ExperimentStatus(data["status"])

        return super().from_dict(data)

    def make_pipeline(
        self,
    ) -> pl.Pipeline | ds.DatasetConfigError | pl.PipelineConfigError:
        ThisPipeline = pl.get_pipeline(self.pipeline)
        if isinstance(ThisPipeline, pl.PipelineNotFound):
            return ThisPipeline

        dataset = ds.get_sequence(self.sequence)
        if isinstance(dataset, ds.SequenceNotFound):
            return dataset

        pipe = ThisPipeline()

        # Set user params
        params = pipe.set_params(self.pipeline_params)
        if len(params) > 0:
            for k, v in params.items():
                print_warning(
                    f"Pipeline {self.name} has unused parameters: {k}={v}. "
                    "Please check your configuration."
                )

        # Set dataset params
        pipe.set_imu_params(dataset.imu_params())
        pipe.set_lidar_params(dataset.lidar_params())
        pipe.set_imu_T_lidar(dataset.imu_T_lidar())

        # Initialize pipeline
        pipe.initialize()

        return pipe
