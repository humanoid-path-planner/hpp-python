"""Compiled regex patterns and primitive constants shared across the package."""

from __future__ import annotations

import re

CLASS_RE = re.compile(
    r"^class\s+([A-Za-z_][A-Za-z0-9_]*)\s*(?:\((?P<bases>[^)]*)\))?\s*:"
)
DEF_RE = re.compile(
    r"^\s*def\s+([A-Za-z_][A-Za-z0-9_]*)\s*\((?P<existing_args>.*?)\)\s*(?:->\s*(?P<existing_ret>.+?))?\s*:"
)
STATICMETHOD_RE = re.compile(r"^\s*@staticmethod\s*$")
PROPERTY_RE = re.compile(r"^\s*@property\s*$")
SETTER_RE = re.compile(r"^\s*@([A-Za-z_][A-Za-z0-9_]*)\.setter\s*$")
DOCSTRING_OPEN_RE = re.compile(r'^\s*"""')
CLASS_ATTR_RE = re.compile(r"^\s*[A-Za-z_][A-Za-z0-9_]*\s*:\s*.+$")

# Boost.Python signature line inside a docstring:
#   addGripper( (Device)arg1, (str)arg2) -> None :
SIG_RE = re.compile(
    r"""^\s*
        (?P<name>[A-Za-z_][A-Za-z0-9_]*)
        \(\s*(?P<args>.*?)\s*\)\s*->\s*
        (?P<ret>[A-Za-z_][A-Za-z0-9_.\[\], ]*?)
        \s*:?\s*$""",
    re.VERBOSE,
)

# A single "(Type)name" argument token
ARG_RE = re.compile(
    r"^\(\s*(?P<type>[A-Za-z_][A-Za-z0-9_.]*)\s*\)\s*(?P<name>[A-Za-z_][A-Za-z0-9_]*)$"
)

BUILTIN_TYPES = {
    "int",
    "float",
    "str",
    "bool",
    "object",
    "list",
    "dict",
    "tuple",
    "set",
    "None",
    "NoneType",
    "bytes",
}
