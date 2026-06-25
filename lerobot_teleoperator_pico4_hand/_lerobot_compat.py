from __future__ import annotations

from collections.abc import Callable
from typing import Any, Generic, TypeAlias, TypeVar

try:
    from lerobot.processor import (  # type: ignore
        RobotAction,
        RobotObservation,
        RobotProcessorPipeline,
        make_default_processors,
    )

    PROCESSOR_API_AVAILABLE = True
except ModuleNotFoundError as exc:
    if exc.name != "lerobot.processor":
        raise

    PROCESSOR_API_AVAILABLE = False

    RobotAction: TypeAlias = dict[str, Any]
    RobotObservation: TypeAlias = dict[str, Any]

    TInput = TypeVar("TInput")
    TOutput = TypeVar("TOutput")

    class RobotProcessorPipeline(Generic[TInput, TOutput]):
        def __init__(self, transform: Callable[[TInput], TOutput]):
            self._transform = transform

        def __call__(self, value: TInput) -> TOutput:
            return self._transform(value)

    def _first_item(value: tuple[RobotAction, RobotObservation]) -> RobotAction:
        return value[0]

    def _identity(value: RobotObservation) -> RobotObservation:
        return value

    def make_default_processors() -> tuple[
        RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
        RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
        RobotProcessorPipeline[RobotObservation, RobotObservation],
    ]:
        return (
            RobotProcessorPipeline(_first_item),
            RobotProcessorPipeline(_first_item),
            RobotProcessorPipeline(_identity),
        )
