"""Small OpenAI-compatible client with auditable timing and retry behavior."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from typing import Any

from openai import OpenAI


@dataclass
class CompletionTrace:
    parsed: dict[str, Any] | None
    raw_text: str
    latency_seconds: float
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int
    attempts: int
    error: str | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class JsonLLMClient:
    """Call Groq or a local OpenAI-compatible endpoint in JSON mode."""

    def __init__(
        self,
        *,
        model: str,
        api_key: str,
        base_url: str,
        timeout: float = 120.0,
        max_attempts: int = 2,
        max_completion_tokens: int = 256,
        request_delay: float = 0.0,
    ) -> None:
        self.model = model
        self.base_url = base_url.rstrip("/")
        self.max_attempts = max(1, max_attempts)
        self.max_completion_tokens = max(32, max_completion_tokens)
        self.request_delay = max(0.0, request_delay)
        self._client = OpenAI(
            api_key=api_key,
            base_url=self.base_url,
            timeout=timeout,
            max_retries=0,
        )
        self._last_request_finished = 0.0

    def _throttle(self) -> None:
        remaining = self.request_delay - (time.perf_counter() - self._last_request_finished)
        if remaining > 0:
            time.sleep(remaining)

    def complete_json(
        self,
        *,
        system: str,
        user: str,
        output_schema: dict[str, Any],
        temperature: float,
    ) -> CompletionTrace:
        """Return parsed JSON plus end-to-end request timing and usage metadata."""
        schema_text = json.dumps(output_schema, ensure_ascii=False, sort_keys=True)
        schema_instruction = (
            "\nReturn exactly one JSON object, with no markdown or surrounding text. "
            f"It must follow this JSON schema: {schema_text}"
        )
        messages = [
            {"role": "system", "content": system + schema_instruction},
            {"role": "user", "content": user},
        ]
        total_latency = 0.0
        total_prompt_tokens = 0
        total_completion_tokens = 0
        total_tokens = 0
        raw_text = ""
        errors: list[str] = []

        for attempt in range(1, self.max_attempts + 1):
            self._throttle()
            started = time.perf_counter()
            try:
                local_options = None
                if "localhost:11434" in self.base_url or "127.0.0.1:11434" in self.base_url:
                    # Ollama's native option is the reliable generation cap in
                    # versions where the OpenAI max_tokens alias is ignored.
                    local_options = {"options": {"num_predict": self.max_completion_tokens}}
                response = self._client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=temperature,
                    response_format={"type": "json_object"},
                    # Ollama 0.11 implements the legacy OpenAI-compatible
                    # ``max_tokens`` field; Groq accepts it as well.
                    max_tokens=self.max_completion_tokens,
                    extra_body=local_options,
                )
                elapsed = time.perf_counter() - started
                self._last_request_finished = time.perf_counter()
                total_latency += elapsed
                usage = response.usage
                if usage is not None:
                    total_prompt_tokens += int(usage.prompt_tokens or 0)
                    total_completion_tokens += int(usage.completion_tokens or 0)
                    total_tokens += int(usage.total_tokens or 0)
                raw_text = response.choices[0].message.content or ""
                parsed = json.loads(raw_text)
                if not isinstance(parsed, dict):
                    raise ValueError("model returned JSON that is not an object")
                return CompletionTrace(
                    parsed=parsed,
                    raw_text=raw_text,
                    latency_seconds=total_latency,
                    prompt_tokens=total_prompt_tokens,
                    completion_tokens=total_completion_tokens,
                    total_tokens=total_tokens,
                    attempts=attempt,
                    error=None,
                )
            except Exception as exc:  # API and parse failures are benchmark outcomes.
                elapsed = time.perf_counter() - started
                self._last_request_finished = time.perf_counter()
                total_latency += elapsed
                errors.append(f"{type(exc).__name__}: {exc}")
                if attempt < self.max_attempts:
                    messages.append(
                        {
                            "role": "user",
                            "content": (
                                "The previous response could not be parsed. Return only one "
                                "valid JSON object following the requested schema."
                            ),
                        }
                    )

        return CompletionTrace(
            parsed=None,
            raw_text=raw_text,
            latency_seconds=total_latency,
            prompt_tokens=total_prompt_tokens,
            completion_tokens=total_completion_tokens,
            total_tokens=total_tokens,
            attempts=self.max_attempts,
            error=" | ".join(errors),
        )
