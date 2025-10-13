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
    """Status of the experiment."""

    Complete = "complete"
    Fail = "fail"
    Started = "started"
    NotRun = "not_run"


@dataclass(kw_only=True)
class Experiment(Metadata):
    """An experiment is a single run of a pipeline on a dataset.

    It contains all the information needed to reproduce the run, including
    the pipeline parameters, dataset, and status.
    """

    name: str
    """Name of the experiment."""
    sequence: str | ds.Dataset
    """Dataset used to run the experiment."""
    sequence_length: int
    """Length of the sequence"""
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
    max_elapsed: Optional[float] = None
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

    @classmethod
    def from_pl_ds(
        cls, pipe: type[pl.Pipeline], ds_obj: ds.Dataset, **kwargs: Any
    ) -> Self:
        """Create an Experiment from a pipeline and dataset.

        Args:
            pipe (type[pl.Pipeline]): The pipeline class.
            ds_obj (ds.Dataset): The dataset object.
            **kwargs: Additional keyword arguments to pass to the Experiment constructor.

        Returns:
            Self: The created Experiment instance.
        """
        return cls(
            name=pipe.name(),
            sequence=ds_obj,
            sequence_length=len(ds_obj),
            pipeline=pipe,
            pipeline_version=pipe.version(),
            pipeline_params=pipe.default_params() | kwargs,
        )

    def setup(
        self,
    ) -> tuple[pl.Pipeline, ds.Dataset] | ds.SequenceNotFound | pl.PipelineNotFound:
        """Setup the experiment by initializing the pipeline and dataset.

        Args:
            self (Experiment): The experiment instance.

        Returns:
            Tuple containing the initialized pipeline and dataset, or an error if the pipeline or dataset could not be found or configured.
        """
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
