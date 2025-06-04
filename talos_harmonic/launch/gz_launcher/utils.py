#!/usr/bin/env python

"""Add utils function."""

from typing import (
    Any,
    Dict,
    Text,
)


def dict_to_string(
        value: Dict[Any, Any],
        *format_args,
        fmt: Text = '{k}: {v}',
        items_separator: Text = '\n',
        **format_kwargs,
) -> Text:
    """Create a simple string version of a dictionnary."""
    return items_separator.join(
        (
            fmt.format(*format_args, k=k, v=v, **format_kwargs)
            for k, v in value.items()
        )
    )
