"""ADVISOR task contracts shared by every benchmark condition.

The benchmark intentionally keeps the task schemas in one place.  Both the
factored and direct controllers receive this exact serialized specification.
"""

from __future__ import annotations

import json
import unicodedata
from typing import Any


OUT_OF_SCOPE = "OutOfScope"
TASK_SPECS: dict[str, dict[str, Any]] = {
    "AddToDatabase": {
        "description": (
            "Register a new user and their daily calorie/macronutrient targets "
            "and intolerances. Do not infer values that were not provided."
        ),
        "parameters": {
            "nome_utente": "lower-case user name (string)",
            "calorie": "daily calories (integer)",
            "proteine": "daily protein grams (integer)",
            "carboidrati": "daily carbohydrate grams (integer)",
            "grassi": "daily fat grams (integer)",
            "intolleranze": "intolerances/allergies (list of lower-case strings)",
        },
        "required": [
            "nome_utente",
            "calorie",
            "proteine",
            "carboidrati",
            "grassi",
        ],
        "defaults": {"intolleranze": []},
    },
    "DishInfo": {
        "description": (
            "Retrieve nutritional, ingredient, allergen, or user-compatibility "
            "information about one specific dish."
        ),
        "parameters": {
            "nome_utente": "lower-case user name when explicitly known (string)",
            "nome_piatto": "one specific lower-case dish name (string)",
            "controllo_ingredienti": (
                "ingredients, allergens, or nutrients to check (list of lower-case strings)"
            ),
        },
        "required": ["nome_piatto"],
        "defaults": {"nome_utente": "", "controllo_ingredienti": []},
    },
    "SubstituteDish": {
        "description": (
            "Propose a meal alternative for a known user, day, and meal, taking "
            "preferences, exclusions, available ingredients, and the weekly-plan "
            "status supplied by the tool context into account."
        ),
        "parameters": {
            "nome_utente": "lower-case user name (string)",
            "ingredienti_rimossi": "ingredients to exclude (list of lower-case strings)",
            "ingredienti_preferiti": "preferred ingredients (list of lower-case strings)",
            "ingredienti_obbligatori_esclusivi": (
                "ingredients explicitly described as the only available ones (list)"
            ),
            "giorno": "Italian weekday without accent (string)",
            "pasto": "one of colazione, pranzo, cena (string)",
            "ha_piano_settimanale": (
                "whether the database reports a weekly plan (boolean from tool_context)"
            ),
        },
        "required": ["nome_utente", "giorno", "pasto", "ha_piano_settimanale"],
        "defaults": {
            "ingredienti_rimossi": [],
            "ingredienti_preferiti": [],
            "ingredienti_obbligatori_esclusivi": [],
        },
    },
}

DECISIONS = ("execute", "clarify", "reject")


def serialized_task_specs() -> str:
    """Return a stable JSON representation injected into every controller."""
    return json.dumps(TASK_SPECS, ensure_ascii=False, sort_keys=True, indent=2)


def empty_parameters(action: str) -> dict[str, Any]:
    """Return defaults for an action without inventing required values."""
    if action not in TASK_SPECS:
        return {}
    return dict(TASK_SPECS[action].get("defaults", {}))


def missing_parameters(action: str, parameters: dict[str, Any]) -> list[str]:
    """Compute syntactically missing slots using the JANUS readiness contract.

    A false weekly-plan flag is a known state, not a missing value.  This avoids
    Python's surprising ``False == 0`` behavior in the ROS node and makes the
    benchmark's definition explicit.
    """
    if action not in TASK_SPECS:
        return []
    missing: list[str] = []
    for name in TASK_SPECS[action]["required"]:
        if name not in parameters:
            missing.append(name)
            continue
        value = parameters[name]
        if value is None or value == "":
            missing.append(name)
        elif isinstance(value, (int, float)) and not isinstance(value, bool) and value == 0:
            missing.append(name)
    return missing


def _plain_text(value: str) -> str:
    value = unicodedata.normalize("NFKD", value.strip().lower())
    return "".join(char for char in value if not unicodedata.combining(char))


def normalize_parameters(action: str, raw: Any) -> dict[str, Any]:
    """Normalize a model-produced parameter dictionary for stable comparison."""
    if action not in TASK_SPECS or not isinstance(raw, dict):
        return {}
    fields = TASK_SPECS[action]["parameters"]
    normalized = empty_parameters(action)
    for name, value in raw.items():
        if name not in fields or value is None:
            continue
        description = fields[name]
        if "(integer)" in description:
            try:
                normalized[name] = int(value)
            except (TypeError, ValueError):
                normalized[name] = 0
        elif "(boolean" in description:
            if isinstance(value, str):
                normalized[name] = value.strip().lower() in {"true", "1", "yes", "si", "sì"}
            else:
                normalized[name] = bool(value)
        elif "(list" in description or description.endswith("(list)"):
            if isinstance(value, str):
                value = [value] if value.strip() else []
            if isinstance(value, list):
                normalized[name] = sorted(
                    {_plain_text(str(item)) for item in value if str(item).strip()}
                )
        elif isinstance(value, str):
            normalized[name] = _plain_text(value)
    return normalized


def normalize_action(value: Any) -> str:
    """Map minor casing differences to a known action."""
    if not isinstance(value, str):
        return OUT_OF_SCOPE
    lookup = {name.lower(): name for name in (*TASK_SPECS.keys(), OUT_OF_SCOPE)}
    return lookup.get(value.strip().lower(), OUT_OF_SCOPE)


def normalize_decision(value: Any) -> str:
    """Map a model decision to the controlled vocabulary."""
    if not isinstance(value, str):
        return "reject"
    aliases = {
        "execute": "execute",
        "proceed": "execute",
        "tool_call": "execute",
        "clarify": "clarify",
        "ask": "clarify",
        "reject": "reject",
        "out_of_scope": "reject",
        "switch_domain": "reject",
    }
    return aliases.get(value.strip().lower(), "reject")
