from evalio.types import Experiment, ExperimentStatus, Param
from evalio.pipelines import Pipeline, register_pipeline

import pytest


class FakePipeline(Pipeline):
    @staticmethod
    def name() -> str:
        return "fake"

    @staticmethod
    def version() -> str:
        return "0.1.0"

    @staticmethod
    def default_params() -> dict[str, Param]:
        return {"param1": 1, "param2": "value"}


def test_serde():
    exp = Experiment(
        name="test",
        status=ExperimentStatus.Complete,
        sequence="newer_college_2020/short_experiment",
        sequence_length=1000,
        pipeline="fake",
        pipeline_version="0.1.0",
        pipeline_params={"param1": 1, "param2": "value"},
        total_elapsed=10.5,
        max_step_elapsed=0.24,
    )
    out = Experiment.from_yaml(exp.to_yaml())
    assert exp == out


def test_verify(capsys: pytest.CaptureFixture[str]):
    register_pipeline(FakePipeline)
    misc = {
        "name": "test",
        "status": ExperimentStatus.Complete,
        "sequence": "newer_college_2020/short_experiment",
        "pipeline_version": "0.1.0",
        "do_verify": True,
    }

    # Bad pipeline name
    with pytest.raises(ValueError) as exc:
        Experiment(
            **misc,  # type: ignore
            sequence_length=1000,
            pipeline="bad_name",
            pipeline_params={"param1": 2, "param2": "value"},  # wrong param1
        )
    assert str(exc.value) == "Experiment 'test' has unknown pipeline 'bad_name'"

    # Bad param name and type
    with pytest.raises(ValueError) as exc:
        Experiment(
            **misc,  # type: ignore
            sequence_length=1000,
            pipeline="fake",
            pipeline_params={"bad_param": 2, "param2": "value"},  # wrong param1
        )
    assert str(exc.value) == "Invalid parameter 'bad_param' for pipeline 'fake'"

    # Bad param type
    with pytest.raises(ValueError) as exc:
        Experiment(
            **misc,  # type: ignore
            sequence_length=1000,
            pipeline="fake",
            pipeline_params={"param1": 2.0, "param2": "value"},  # wrong param1
        )
    assert (
        str(exc.value)
        == "Invalid type for parameter 'param1' for pipeline 'fake': expected 'int', got 'float'"
    )

    # Too long length
    Experiment(
        **misc,  # type: ignore
        sequence_length=2000000,
        pipeline="fake",
        pipeline_params={"param1": 1, "param2": "value"},
    )

    captured = capsys.readouterr()
    assert (
        captured.out
        == "Warning: Experiment 'test' has sequence_length 2000000 > dataset length 15302, reducing to 15302\n"
    )
