from rich.console import Console


def print_warning(warn: str):
    """
    Print a warning message.
    """
    Console(soft_wrap=True).print(f"[bold red]Warning[/bold red]: {warn}")
