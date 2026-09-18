"""Text normalisation applied before rule parsing.

The original utterance is always preserved verbatim (it goes into the run's
``input.json`` and event log); this module only produces a comparable form.
Normalisation is intentionally conservative -- it must not invent words.
"""

from __future__ import annotations

import re
import unicodedata

_WHITESPACE = re.compile(r"\s+")

# Punctuation that carries no instruction meaning for the supported vocabulary.
_PUNCTUATION = "，。！？；：、,.!?;:()（）【】[]{}<>\"'“”‘’《》"


def normalize(text: str) -> str:
    """Full-width -> half-width, lower-cased, punctuation -> space, collapsed."""
    if not isinstance(text, str):
        raise TypeError("text must be a string")
    folded = unicodedata.normalize("NFKC", text).lower()
    for char in _PUNCTUATION:
        folded = folded.replace(char, " ")
    return _WHITESPACE.sub(" ", folded).strip()
