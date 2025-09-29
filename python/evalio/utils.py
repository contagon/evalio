from enum import Enum, auto
from rich.console import Console


def print_warning(warn: str):
    """
    Print a warning message.
    """
    Console(soft_wrap=True).print(f"[bold red]Warning[/bold red]: {warn}")


# For converting dataset names to snake case
class CharKinds(Enum):
    LOWER = auto()
    UPPER = auto()
    DIGIT = auto()
    OTHER = auto()

    @staticmethod
    def from_char(char: str):
        if char.islower():
            return CharKinds.LOWER
        if char.isupper():
            return CharKinds.UPPER
        if char.isdigit():
            return CharKinds.DIGIT
        return CharKinds.OTHER


def pascal_to_snake(identifier: str) -> str:
    """Convert a PascalCase identifier to snake_case.

    Args:
        identifier (str): The PascalCase identifier to convert.

    Returns:
        str: The converted snake_case identifier.
    """
    # Only split when going from lower to something else
    # this handles digits better than other approaches
    splits: list[int] = []
    last_kind = CharKinds.from_char(identifier[0])
    for i, char in enumerate(identifier[1:], start=1):
        kind = CharKinds.from_char(char)
        if last_kind == CharKinds.LOWER and kind != CharKinds.LOWER:
            splits.append(i)
        last_kind = kind

    parts = [identifier[i:j] for i, j in zip([0] + splits, splits + [None])]
    return "_".join(parts).lower()
