from functools import wraps
import typer

import docstring_parser

import inspect  # for python 3.9 +
from typing import Callable, Any, Annotated, get_origin, get_args


def from_docstring(command: Callable[..., Any]) -> Callable[..., Any]:
    """
    A decorator that applies help texts from the function's docstring to Typer arguments/options.

    It will only apply the help text if no help text has been explicitly set in the Typer argument/option.

    Args:
        command (Callable): The function to decorate.

    Returns:
        Callable: The decorated function with help texts applied, without overwriting existing settings.
    """

    if command.__doc__ is None:
        return command

    # Parse the docstring and extract parameter descriptions
    docstring = docstring_parser.parse(command.__doc__)
    param_help = {param.arg_name: param.description for param in docstring.params}

    # The commands's full help text (summary + long description)
    command_help = (
        f"{docstring.short_description or ''}\n\n{docstring.long_description or ''}"
    )

    # Get the signature of the original function
    sig = inspect.signature(command)
    parameters = sig.parameters

    @wraps(command)
    def wrapper(**kwargs: Any) -> Any:
        return command(**kwargs)

    # Prepare a new mapping for parameters
    new_parameters: list[inspect.Parameter] = []

    for name, param in parameters.items():
        help_text = param_help.get(
            name, ""
        )  # Get help text from docstring if available

        param_type = (
            param.annotation if param.annotation is not inspect.Parameter.empty else str
        )  # Default to str if no annotation

        # Handle Annotated (e.g., Annotated[int, typer.Argument()] or Annotated[str, typer.Option()])
        # Check if the parameter uses Annotated
        if get_origin(param_type) is Annotated:
            param_type, *metadata = get_args(param_type)
            # Iterate through the metadata to find Typer's Argument or Option
            new_metadata: list[Any] = []
            for m in metadata:
                if isinstance(m, (typer.models.ArgumentInfo, typer.models.OptionInfo)):
                    # Only add help text if it's not already set
                    if not m.help:
                        m.help = help_text
                new_metadata.append(m)

            # Rebuild the annotated type with updated metadata
            new_param = param.replace(annotation=Annotated[param_type, *new_metadata])
            # for python 3.8, this is not perfect ...
            # new_param = param.replace(annotation=Annotated[param_type, new_metadata[0]])

        # If it's an Option or Argument directly (e.g., a: int = typer.Option(...))
        elif isinstance(
            param.default, (typer.models.ArgumentInfo, typer.models.OptionInfo)
        ):
            if not param.default.help:
                param.default.help = help_text
            new_param = param

        else:
            # If the parameter has no default, treat it as an Argument
            if param.default is inspect.Parameter.empty:
                new_param = param.replace(
                    default=typer.Argument(..., help=help_text, show_default=False),
                    annotation=param_type,
                )
            else:
                # If the parameter has a default, treat it as an Option
                new_param = param.replace(
                    default=typer.Option(
                        param.default, help=help_text, show_default=False
                    ),
                    annotation=param_type,
                )

        new_parameters.append(new_param)

    # Create a new signature with updated parameters
    new_sig = sig.replace(parameters=new_parameters)

    # Apply the new signature to the wrapper function
    wrapper.__signature__ = new_sig  # type: ignore

    wrapper.__doc__ = command_help.strip()

    return wrapper
