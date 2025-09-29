"""
These are extended types that do depend on other parts of evalio.
"""

from __future__ import annotations

from enum import Enum
from dataclasses import dataclass, InitVar
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
    do_verify: InitVar[bool] = False
    """If true, verify the experiment parameters are valid."""

    def __post_init__(self, do_verify: bool):
        if do_verify:
            self.verify()

    def verify(self):
        # Verify pipeline is good
        ThisPipeline = pl.get_pipeline(self.pipeline)
        if isinstance(ThisPipeline, pl.PipelineNotFound):
            raise ValueError(
                f"Experiment '{self.name}' has unknown pipeline '{self.pipeline}'"
            )

        all_params = ThisPipeline.default_params()
        for key in self.pipeline_params.keys():
            if key not in all_params:
                raise ValueError(
                    f"Invalid parameter '{key}' for pipeline '{ThisPipeline.name()}'"
                )
            elif key in all_params and not isinstance(
                self.pipeline_params[key], type(all_params[key])
            ):
                raise ValueError(
                    f"Invalid type for parameter '{key}' for pipeline '{ThisPipeline.name()}': "
                    f"expected '{type(all_params[key]).__name__}', got '{type(self.pipeline_params[key]).__name__}'"
                )

        # Verify dataset is good
        dataset = ds.get_sequence(self.sequence)
        if isinstance(dataset, ds.SequenceNotFound):
            raise ValueError(
                f"Experiment {self.name} has unknown dataset {self.sequence}"
            )

        if self.sequence_length > (length := len(dataset)):
            print_warning(
                f"Experiment '{self.name}' has sequence_length {self.sequence_length} > dataset length {len(dataset)}, reducing to {len(dataset)}"
            )
            self.sequence_length = length

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        if "status" in data:
            data["status"] = ExperimentStatus(data["status"])

        return super().from_dict(data)

    def make_pipeline(self) -> pl.Pipeline:
        ThisPipeline = pl.get_pipeline(self.pipeline)
        if isinstance(ThisPipeline, pl.PipelineNotFound):
            raise ValueError(
                f"Experiment {self.name} has unknown pipeline {self.pipeline}"
            )

        dataset = ds.get_sequence(self.sequence)
        if isinstance(dataset, ds.SequenceNotFound):
            raise ValueError(
                f"Experiment {self.name} has unknown dataset {self.sequence}"
            )

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
