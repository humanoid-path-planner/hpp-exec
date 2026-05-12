"""Lightweight segment data structures."""

from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Segment:
    """A trajectory segment with optional pre/post actions.

    Actions are callables that return True on success, False on failure.
    Any callable works: bound methods, lambdas, or free functions.
    """

    start_index: int
    """Start config index (inclusive)."""

    end_index: int
    """End config index (exclusive)."""

    pre_actions: list[Callable[[], bool]] = field(default_factory=list)
    """Actions to run before sending this segment's trajectory."""

    post_actions: list[Callable[[], bool]] = field(default_factory=list)
    """Actions to run after this segment's trajectory completes."""
