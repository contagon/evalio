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
    NotRun = "not_run"


@dataclass(kw_only=True)
class Experiment(Metadata):
    name: str
    """Name of the experiment."""
    sequence: str | ds.Dataset
    """Dataset used to run the experiment."""
    sequence_length: int
    """Length of the sequence, if set"""
    pipeline: str | type[pl.Pipeline]
    """Pipeline used to generate the trajectory."""
    pipeline_version: str
    """Version of the pipeline used."""
    pipeline_params: dict[str, Param]
    """Parameters used for the pipeline."""
    status: ExperimentStatus = ExperimentStatus.NotRun
    """Status of the experiment, e.g. "success", "failure", etc."""
    total_elapsed: Optional[float] = None
    """Total time taken for the experiment, as a string."""
    max_step_elapsed: Optional[float] = None
    """Maximum time taken for a single step in the experiment, as a string."""

    def to_dict(self) -> dict[str, Any]:
        d = super().to_dict()
        d["status"] = self.status.value
        if isinstance(self.pipeline, type):
            d["pipeline"] = self.pipeline.name()
        if isinstance(self.sequence, ds.Dataset):
            d["sequence"] = self.sequence.full_name

        return d

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        if "status" in data:
            data["status"] = ExperimentStatus(data["status"])
        else:
            data["status"] = ExperimentStatus.Started

        return super().from_dict(data)

    def setup(
        self,
    ) -> (
        tuple[pl.Pipeline, ds.Dataset] | ds.DatasetConfigError | pl.PipelineConfigError
    ):
        if isinstance(self.pipeline, str):
            ThisPipeline = pl.get_pipeline(self.pipeline)
            if isinstance(ThisPipeline, pl.PipelineNotFound):
                return ThisPipeline
        else:
            ThisPipeline = self.pipeline

        if isinstance(self.sequence, ds.Dataset):
            dataset = self.sequence
        else:
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

        return pipe, dataset
