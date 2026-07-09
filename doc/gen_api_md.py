#!/usr/bin/env python3
"""Generate Markdown API reference from pyhpp .pyi stubs for mdbook.

Usage:
    gen_api_md.py --stubs <path/to/site-packages> \
                  --output <path/to/mdbook/src/reference/hpp-python/api>
Exemple:
    python gen_api_md.py \
            --stubs $DEVEL_HPP_DIR/install/lib/python3.13/site-packages \
            --output $DEVEL_HPP_DIR/src/hpp-doc/mdbook/src/reference/hpp-python/api/
"""

import ast
import keyword
import re
import sys
import argparse
from pathlib import Path

BOOST_SIG_RE = re.compile(r"^[\w][\w:]*\s*\(.*\)\s*->\s*\S")
BOOST_RET_RE = re.compile(r"->\s*(\S+?)(?:\s*:)?\s*$")

SKIP_NAMES = {"__reduce__", "__instance_size__", "__hash__"}
SKIP_BASES = {"object", "instance"}

NOT_INSTANTIABLE = re.compile(
    r"raises an exception|cannot be instantiated from python", re.IGNORECASE
)
PARAM_RE = re.compile(r"^:param\s+(\w+):\s*(.*)")
RETURNS_RE = re.compile(r"^:returns?:\s*(.*)")
TYPE_RE = re.compile(r"^:(?:type|rtype)\b")
SECTION_RE = re.compile(
    r"^(inputs?|outputs?|parameters?|args?|arguments?)s?:?\s*$", re.IGNORECASE
)
KV_RE = re.compile(r"^([A-Za-z]\w*): +(.+)$")

MODULES = [
    (
        "pyhpp/core/bindings.pyi",
        "pyhpp.core",
        "Path planning problem, planners, optimizers, roadmap, steering methods.",
    ),
    (
        "pyhpp/core/path/bindings.pyi",
        "pyhpp.core.path",
        "Concrete path types: StraightPath, PathVector, splines.",
    ),
    (
        "pyhpp/core/path_optimization/bindings.pyi",
        "pyhpp.core.path_optimization",
        "Concrete path optimizer implementations.",
    ),
    (
        "pyhpp/core/problem_target/bindings.pyi",
        "pyhpp.core.problem_target",
        "Problem target types (goal configurations, etc.).",
    ),
    (
        "pyhpp/constraints/bindings.pyi",
        "pyhpp.constraints",
        "Differentiable functions, implicit/explicit constraints, hierarchical solver.",
    ),
    (
        "pyhpp/manipulation/bindings.pyi",
        "pyhpp.manipulation",
        "Manipulation-specific Device, constraint graph, path planners.",
    ),
    (
        "pyhpp/manipulation/urdf/bindings.pyi",
        "pyhpp.manipulation.urdf",
        "URDF/SRDF loading for manipulation devices.",
    ),
    (
        "pyhpp/manipulation/steering_method/bindings.pyi",
        "pyhpp.manipulation.steering_method",
        "Manipulation-specific steering methods.",
    ),
    (
        "pyhpp/pinocchio/bindings.pyi",
        "pyhpp.pinocchio",
        "Robot model (Device), Lie-group utilities.",
    ),
    (
        "pyhpp/pinocchio/urdf/bindings.pyi",
        "pyhpp.pinocchio.urdf",
        "URDF/SRDF loading for pinocchio devices.",
    ),
]


def slug(module_name: str) -> str:
    return module_name.replace(".", "-").removeprefix("pyhpp-")


def simplify_type(name: str) -> str:
    return name.split(".")[-1]


# ---------------------------------------------------------------------------
# Docstring line formatting
# ---------------------------------------------------------------------------


def doxygen_to_katex(text: str) -> str:
    """Convert Doxygen LaTeX notation to mdbook-katex syntax."""
    # \begin{env}...\end{env} → $$\begin{env}...\end{env}$$
    text = re.sub(
        r"\\begin\{(\w+\*?)\}(.*?)\\end\{\1\}",
        lambda m: f"\n$$\\begin{{{m.group(1)}}}{m.group(2)}\\end{{{m.group(1)}}}$$\n",
        text,
        flags=re.DOTALL,
    )
    return text


def _format_line(s: str) -> str | None:
    if TYPE_RE.match(s):
        return None
    m = PARAM_RE.match(s)
    if m:
        return f"**{m.group(1)}** — {m.group(2)}"
    m = RETURNS_RE.match(s)
    if m:
        return f"**Returns** — {m.group(1)}"
    if SECTION_RE.match(s):
        return f"*{s.rstrip(':').capitalize()}:*"
    m = KV_RE.match(s)
    if m and " " not in m.group(1):
        return f"**{m.group(1)}** — {m.group(2)}"
    return s.replace("<", "&lt;").replace(">", "&gt;").replace("|", "\\|")


def text_to_cell(text: str) -> str:
    """Flatten multiline text into a single table cell using <br>."""
    lines = []
    for line in text.split("\n"):
        s = line.strip()
        if not s:
            continue
        # List markers don't render inside table cells
        s = re.sub(r"^[-*]\s+", "", s)
        lines.append(s)
    return " <br> ".join(lines)


# ---------------------------------------------------------------------------
# Overload-aware docstring parsing
# ---------------------------------------------------------------------------


def parse_overloads(raw: str | None) -> list[tuple[str | None, str]]:
    """Split a Boost.Python docstring into (return_type, formatted_text) pairs."""
    if not raw:
        return []

    result: list[tuple[str | None, str]] = []
    current_ret: str | None = None
    current: list[str] = []

    for line in raw.splitlines():
        s = line.strip()
        if BOOST_SIG_RE.match(s):
            if current:
                text = "\n".join(current).rstrip()
                if text:
                    result.append((current_ret, text))
                current = []
            m = BOOST_RET_RE.search(s)
            current_ret = m.group(1) if m else None
        elif not s:
            if current and current[-1] != "":
                current.append("")
        else:
            formatted = _format_line(s)
            if formatted is not None:
                current.append(formatted)

    if current:
        text = "\n".join(current).rstrip()
        if text:
            result.append((current_ret, text))

    return result


def overloads_to_cell(overloads: list[tuple[str | None, str]]) -> str:
    """Convert overloads to a single table cell string."""
    if not overloads:
        return ""

    if len(overloads) == 1:
        return text_to_cell(overloads[0][1])

    # 2 overloads — try getter/setter
    if len(overloads) == 2:
        rets = [r for r, _ in overloads]
        if sum(1 for r in rets if r == "None") == 1:
            parts = []
            for ret, text in overloads:
                label = "**setter**" if ret == "None" else "**getter**"
                parts.append(f"{label} — {text_to_cell(text)}")
            return " <br> ".join(parts)

    # N overloads — numbered
    parts = []
    for i, (_, text) in enumerate(overloads, 1):
        parts.append(f"**{i}.** {text_to_cell(text)}")
    return " <br> ".join(parts)


# ---------------------------------------------------------------------------
# AST helpers
# ---------------------------------------------------------------------------


def get_docstring(node: ast.AST) -> str | None:
    if (
        node.body
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and isinstance(node.body[0].value.value, str)
    ):
        return node.body[0].value.value
    return None


def is_ellipsis_body(node: ast.FunctionDef) -> bool:
    return (
        len(node.body) == 1
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and node.body[0].value.value is ...
    )


def has_decorator(node: ast.FunctionDef, name: str) -> bool:
    for d in node.decorator_list:
        if isinstance(d, ast.Name) and d.id == name:
            return True
        if isinstance(d, ast.Attribute) and d.attr == name:
            return True
    return False


def is_setter(node: ast.FunctionDef) -> bool:
    return any(
        isinstance(d, ast.Attribute) and d.attr == "setter" for d in node.decorator_list
    )


def base_links(
    class_node: ast.ClassDef, stub_module_index: dict[str, str]
) -> list[str]:
    """Return base class names as markdown links with full dotted path displayed.

    stub_module_index maps dotted stub module path to page slug,
    e.g. "pyhpp.core.bindings" -> "core".
    """
    links = []
    for b in class_node.bases:
        raw = (
            ast.unparse(b)
            if isinstance(b, ast.Attribute)
            else (b.id if isinstance(b, ast.Name) else "")
        )
        if not raw:
            continue
        class_name = simplify_type(raw)  # last dotted component
        if class_name in SKIP_BASES:
            continue
        # Try to resolve the page from the module path (all components but last)
        module_path = raw.rsplit(".", 1)[0] if "." in raw else ""
        page = stub_module_index.get(module_path) or stub_module_index.get(raw)
        if page:
            anchor = class_name.lower()
            links.append(f"[`{raw}`]({page}.md#{anchor})")
        else:
            links.append(f"`{raw}`")
    return links


# ---------------------------------------------------------------------------
# Markdown rendering
# ---------------------------------------------------------------------------


def collect_member_row(node: ast.FunctionDef) -> tuple[str, str, str] | None:
    """Return (name, tag, cell_text) or None if nothing to show."""
    if node.name in SKIP_NAMES or is_ellipsis_body(node) or is_setter(node):
        return None

    raw = get_docstring(node)

    if node.name == "__init__":
        return None  # handled separately as class-level note

    overloads = parse_overloads(raw)
    if not overloads:
        return None

    is_prop = has_decorator(node, "property")
    tag = " *(property)*" if is_prop else ""
    cell = overloads_to_cell(overloads)
    return (node.name, tag, cell)


def render_class(
    class_node: ast.ClassDef, lines: list[str], stub_module_index: dict[str, str]
) -> bool:
    raw_doc = get_docstring(class_node)
    bases = base_links(class_node, stub_module_index)

    # Collect method rows
    rows: list[tuple[str, str, str]] = []
    init_not_instantiable = False
    for item in class_node.body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if item.name == "__init__":
                init_raw = get_docstring(item)
                if NOT_INSTANTIABLE.search(init_raw or ""):
                    init_not_instantiable = True
            else:
                row = collect_member_row(item)
                if row:
                    rows.append(row)

    if not raw_doc and not rows and not init_not_instantiable:
        return False

    lines.append(f"## `{class_node.name}`\n")

    meta: list[str] = []
    if bases:
        meta.append(f"*Inherits: {', '.join(bases)}*")
    if init_not_instantiable:
        meta.append("*Not instantiable from Python.*")
    if meta:
        lines.append("  ".join(meta) + "\n")

    if raw_doc:
        overloads = parse_overloads(raw_doc)
        doc = overloads_to_cell(overloads)
        if doc:
            doc = doxygen_to_katex(doc.replace(" <br> ", "\n\n"))
            quoted = "\n".join(
                f"> {ln}" if ln.strip() else ">" for ln in doc.splitlines()
            )
            lines.append(quoted + "\n")

    if rows:
        lines.append("| Method | Description |")
        lines.append("|:---|:---|")
        for name, tag, cell in rows:
            lines.append(f"| `{name}`{tag} | {cell} |")
        lines.append("")

    lines.append("---\n")
    return True


def render_function_row(func_node: ast.FunctionDef) -> tuple[str, str] | None:
    """Return (name, cell_text) for a module-level function."""
    if func_node.name in SKIP_NAMES or is_ellipsis_body(func_node):
        return None
    overloads = parse_overloads(get_docstring(func_node))
    cell = overloads_to_cell(overloads)
    if not cell:
        return None
    return (func_node.name, cell)


# ---------------------------------------------------------------------------
# Per-module generation
# ---------------------------------------------------------------------------

_KW_PARAM_RE = re.compile(
    r"([,(]\s*)(" + "|".join(re.escape(kw) for kw in keyword.kwlist) + r")(\s*:)"
)


def _parse_stub(stub_path: Path) -> ast.Module | None:
    try:
        source = stub_path.read_text()
        source = re.sub(r"ClassVar\[[^\]]*\.\]", "ClassVar[Any]", source)
        source = _KW_PARAM_RE.sub(r"\1\2_\3", source)
        return ast.parse(source)
    except Exception:
        return None


def build_stub_module_index() -> dict[str, str]:
    """Map dotted stub module path -> page_slug.

    e.g. "pyhpp.core.bindings" -> "core", used to resolve base class links
    like pyhpp.core.bindings.Problem -> core.md#problem without ambiguity."""
    index: dict[str, str] = {}
    for rel_stub, module_name, _ in MODULES:
        # "pyhpp/core/bindings.pyi" -> "pyhpp.core.bindings"
        dotted = str(Path(rel_stub).with_suffix("")).replace("/", ".")
        index[dotted] = slug(module_name)
    return index


def generate_module_md(
    stub_path: Path, module_name: str, stub_module_index: dict[str, str]
) -> str:
    tree = _parse_stub(stub_path)
    if tree is None:
        return f"# `{module_name}`\n\n*Failed to parse stub.*\n"

    lines: list[str] = [f"# `{module_name}`\n"]
    has_content = False

    func_rows: list[tuple[str, str]] = []

    for item in tree.body:
        if isinstance(item, ast.ClassDef):
            if render_class(item, lines, stub_module_index):
                has_content = True
        elif isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            row = render_function_row(item)
            if row:
                func_rows.append(row)

    if func_rows:
        lines.append("## Functions\n")
        lines.append("| Function | Description |")
        lines.append("|:---|:---|")
        for name, cell in func_rows:
            lines.append(f"| `{name}()` | {cell} |")
        lines.append("")
        has_content = True

    if not has_content:
        lines.append("*No documented symbols in this module.*\n")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Generate mdbook Markdown pages from pyhpp .pyi stubs"
    )
    parser.add_argument(
        "--stubs",
        required=True,
        type=Path,
        help="Python site-packages directory (e.g. install/lib/python3.x/site-packages)",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Output directory for generated .md files",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    stub_module_index = build_stub_module_index()

    generated: list[tuple[str, str, str]] = []
    for rel_stub, module_name, description in MODULES:
        stub_path = args.stubs / rel_stub
        if not stub_path.exists():
            print(f"WARNING: {stub_path} not found, skipping", file=sys.stderr)
            continue
        out_file = args.output / f"{slug(module_name)}.md"
        out_file.write_text(
            generate_module_md(stub_path, module_name, stub_module_index)
        )
        generated.append((module_name, slug(module_name), description))
        print(f"  {out_file.name}")

    index = [
        "# pyhpp API Reference\n\n",
        "Python bindings for HPP — auto-generated from `.pyi` stubs.\n\n",
        "| Module | Description |\n",
        "|:---|:---|\n",
    ]
    for mod, s, desc in generated:
        index.append(f"| [`{mod}`]({s}.md) | {desc} |\n")
    (args.output / "index.md").write_text("".join(index))
    print("  index.md")


if __name__ == "__main__":
    main()
