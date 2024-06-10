from enum import Enum


class ExperimentType(Enum):
    """Enum class for the different types of experiments."""

    NULL_SPACE = 1
    FOLLOW_TRAJECTORY = 2
    MONTE_CARLO = 3


class ExperimentStatus(Enum):
    """Enum class for the different statuses of an experiment."""

    SUCCESS = 1
    FAILED = 2
