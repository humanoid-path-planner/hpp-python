"""IR (Intermediate Representation) data models for a parsed .pyi file.

Each model knows how to render itself to valid Python stub syntax via __str__.
"""

from __future__ import annotations

from ._text_utils import indent_block
from ._type_utils import quote_type


class Overload:
    """A single signature variant (useful when a docstring contains several overloads)."""

    def __init__(
        self, ret: str, params: list[tuple[str, str]], description: str, is_method: bool
    ):
        self.ret = ret
        self.params = params  # "self" already stripped when is_method=True
        self.description = description
        self.is_method = is_method


class Function:
    def __init__(self):
        self.name = ""
        self.overloads: list[Overload] = []
        self.from_docstring = False
        self.raw_blocks: list[list[str]] = []
        self.is_property = False
        self.setter_overloads: list[Overload] = []
        self.setter_raw_blocks: list[list[str]] = []
        self.setter_from_override = False
        self.in_class = True

    # -- backward-compat shims for single-block/single-overload access --
    @property
    def raw_lines(self):
        return self.raw_blocks[0] if self.raw_blocks else []

    @raw_lines.setter
    def raw_lines(self, value):
        self.raw_blocks = [value] if value else []

    @property
    def type_func(self):
        return self.overloads[0].ret if self.overloads else ""

    @property
    def param(self):
        return self.overloads[0].params if self.overloads else []

    @property
    def is_method(self):
        return self.overloads[0].is_method if self.overloads else False

    @property
    def docstring(self):
        return self.overloads[0].description if self.overloads else ""

    def _render_one(
        self, ov: Overload, overload_decorator: bool, decorator: str = ""
    ) -> str:
        param_names = ["self"] if ov.is_method else []
        param_names += [f"{n}: {quote_type(t)}" for n, t in ov.params]
        params_str = ", ".join(param_names)
        ret = quote_type(ov.ret) if ov.ret else "None"
        prefix = ""
        if overload_decorator:
            prefix += "@typing.overload\n"
        if decorator:
            prefix += decorator + "\n"
        elif not ov.is_method and self.in_class:
            # @staticmethod n'a de sens qu'a l'interieur d'une classe;
            # une fonction de niveau module ne doit jamais le recevoir
            prefix += "@staticmethod\n"
        header = f"{prefix}def {self.name}({params_str}) -> {ret}:"
        body = f'"""\n{ov.description}\n"""' if ov.description else "..."
        return header + "\n" + indent_block(body, 4)

    def __str__(self):
        if not self.from_docstring:
            all_blocks = list(self.raw_blocks)
            # si un setter a ete rencontre sans docstring parsable, rendre ses lignes brutes
            # (sauf si setter_overloads est deja rempli via SETTER_TYPE_OVERRIDES, pour eviter un doublon)
            if not self.setter_overloads:
                all_blocks += self.setter_raw_blocks
            multi = len(all_blocks) > 1
            parts = []
            for block in all_blocks:
                indents = [len(ln) - len(ln.lstrip()) for ln in block if ln.strip()]
                min_indent = min(indents) if indents else 0
                dedented = [
                    ln[min_indent:] if len(ln) >= min_indent else ln for ln in block
                ]
                already_has_overload = any(
                    ln.strip() == "@typing.overload" for ln in dedented
                )
                is_property_pair = any(
                    ln.strip() == "@property" or ln.strip().endswith(".setter")
                    for ln in dedented
                )
                if multi and not already_has_overload and not is_property_pair:
                    dedented = ["@typing.overload"] + dedented
                parts.append("\n".join(dedented))
            rendered = "\n\n".join(parts)

            # getter sans docstring parsable mais setter type via override:
            # on ajoute quand meme le setter type, sinon l'override serait silencieusement perdu
            if self.setter_overloads:
                multi_set = len(self.setter_overloads) > 1
                setter_parts = [
                    self._render_one(
                        ov,
                        overload_decorator=multi_set,
                        decorator=f"@{self.name}.setter",
                    )
                    for ov in self.setter_overloads
                ]
                rendered = rendered + "\n\n" + "\n\n".join(setter_parts)
            return rendered

        if self.is_property:
            parts = []
            multi_get = len(self.overloads) > 1
            for ov in self.overloads:
                parts.append(
                    self._render_one(
                        ov, overload_decorator=multi_get, decorator="@property"
                    )
                )
            multi_set = len(self.setter_overloads) > 1
            for ov in self.setter_overloads:
                parts.append(
                    self._render_one(
                        ov,
                        overload_decorator=multi_set,
                        decorator=f"@{self.name}.setter",
                    )
                )
            if not self.setter_overloads:
                for block in self.setter_raw_blocks:
                    indents = [len(ln) - len(ln.lstrip()) for ln in block if ln.strip()]
                    min_indent = min(indents) if indents else 0
                    dedented = [
                        ln[min_indent:] if len(ln) >= min_indent else ln for ln in block
                    ]
                    parts.append("\n".join(dedented))
            return "\n\n".join(parts)

        multi = len(self.overloads) > 1
        return "\n\n".join(
            self._render_one(ov, overload_decorator=multi) for ov in self.overloads
        )

    def __repr__(self):
        return self.__str__()


class ClassI:
    def __init__(self, is_module: bool = False):
        self.name = ""
        self.bases: list[str] = []
        self.docstring = ""
        self.functions: list[Function] = []
        self.nested_classes: list["ClassI"] = []
        self.is_module = is_module
        self.extra_lines: list[str] = []

    def __str__(self):
        if self.is_module:
            parts = []
            for fn in self.functions:
                parts.append(str(fn))
                parts.append("")
            return "\n".join(parts)

        bases_str = f"({', '.join(self.bases)})" if self.bases else ""
        lines = [f"class {self.name}{bases_str}:"]
        if self.docstring:
            lines.append(indent_block(f'"""\n{self.docstring}\n"""', 4))

        # __slots__ = () ferme la classe a toute assignation d'attribut arbitraire tout en
        # heritant correctement des slots declares par les classes de base. On ne liste jamais
        # les @property dans __slots__: avoir les deux pour le meme nom est un conflit Python
        # (ValueError a la definition) et cause des faux positifs Pyright.
        lines.append(indent_block("__slots__ = ()", 4))

        for extra in self.extra_lines:
            lines.append(indent_block(extra, 4))

        for nested in self.nested_classes:
            lines.append(indent_block(str(nested), 4))
            lines.append("")

        for fn in self.functions:
            lines.append(indent_block(str(fn), 4))
            lines.append("")
        return "\n".join(lines)

    def __repr__(self):
        return self.__str__()
