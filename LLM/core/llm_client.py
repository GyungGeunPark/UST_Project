# LLM API Client for Robot Control

import os
import time
import asyncio
import logging
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class LLMStats:
    """LLM usage statistics"""
    total_requests: int = 0
    total_prompt_tokens: int = 0
    total_completion_tokens: int = 0
    total_latency_ms: float = 0.0

    @property
    def average_latency_ms(self) -> float:
        if self.total_requests == 0:
            return 0.0
        return self.total_latency_ms / self.total_requests

    @property
    def estimated_cost_usd(self) -> float:
        # Rough estimate based on GPT-4 pricing
        input_cost = (self.total_prompt_tokens / 1000) * 0.01
        output_cost = (self.total_completion_tokens / 1000) * 0.03
        return input_cost + output_cost


@dataclass
class CacheEntry:
    """Cache entry for LLM responses"""
    command: str
    response: Dict[str, Any]
    timestamp: float
    hits: int = 0


class CommandCache:
    """Simple cache for LLM command responses"""

    def __init__(self, max_size: int = 100, ttl: float = 3600.0):
        self.max_size = max_size
        self.ttl = ttl
        self._cache: Dict[str, CacheEntry] = {}
        self._total_hits = 0
        self._total_requests = 0

    def get(self, command: str) -> Optional[Dict[str, Any]]:
        """Get cached response for command"""
        self._total_requests += 1

        # Normalize command
        normalized = self._normalize_command(command)

        if normalized in self._cache:
            entry = self._cache[normalized]

            # Check TTL
            if time.time() - entry.timestamp > self.ttl:
                del self._cache[normalized]
                return None

            entry.hits += 1
            self._total_hits += 1
            return entry.response

        return None

    def set(self, command: str, response: Dict[str, Any]):
        """Cache a response"""
        # Evict oldest if full
        if len(self._cache) >= self.max_size:
            oldest_key = min(self._cache.keys(),
                           key=lambda k: self._cache[k].timestamp)
            del self._cache[oldest_key]

        normalized = self._normalize_command(command)
        self._cache[normalized] = CacheEntry(
            command=command,
            response=response,
            timestamp=time.time()
        )

    def clear(self):
        """Clear the cache"""
        self._cache.clear()

    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics"""
        return {
            "size": len(self._cache),
            "max_size": self.max_size,
            "hit_rate": self._total_hits / max(1, self._total_requests),
            "total_hits": self._total_hits
        }

    def _normalize_command(self, command: str) -> str:
        """Normalize command for cache key"""
        return command.strip().lower()


class LLMClient:
    """Client for LLM API calls"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.provider = config.get("provider", "openai")

        # Rate limiting
        rate_config = config.get("rate_limit", {})
        self._min_interval = rate_config.get("min_interval", 0.1)
        self._max_retries = rate_config.get("max_retries", 3)
        self._retry_delay = rate_config.get("retry_delay", 1.0)
        self._last_call_time = 0.0

        # Cache
        cache_config = config.get("cache", {})
        self._cache_enabled = cache_config.get("enabled", True)
        self._cache = CommandCache(
            max_size=cache_config.get("max_size", 100),
            ttl=cache_config.get("ttl", 3600.0)
        )

        # Statistics
        self.stats = LLMStats()

        # Initialize client
        self._client = None
        self._init_client()

    def _init_client(self):
        """Initialize the LLM client based on provider"""
        if self.provider == "openai":
            self._init_openai()
        elif self.provider == "anthropic":
            self._init_anthropic()
        else:
            raise ValueError(f"Unknown provider: {self.provider}")

    def _init_openai(self):
        """Initialize OpenAI client"""
        try:
            from openai import AsyncOpenAI

            openai_config = self.config.get("openai", {})
            api_key = openai_config.get("api_key", "")

            # Resolve environment variable
            if api_key.startswith("${") and api_key.endswith("}"):
                env_var = api_key[2:-1]
                api_key = os.environ.get(env_var, "")

            if not api_key:
                api_key = os.environ.get("OPENAI_API_KEY", "")

            self._client = AsyncOpenAI(api_key=api_key)
            self._model = openai_config.get("model", "gpt-4o")
            self._temperature = openai_config.get("temperature", 0.1)
            self._max_tokens = openai_config.get("max_tokens", 1024)
            self._timeout = openai_config.get("timeout", 30.0)

            logger.info(f"OpenAI client initialized with model: {self._model}")

        except ImportError:
            logger.error("OpenAI package not installed. Run: pip install openai")
            raise

    def _init_anthropic(self):
        """Initialize Anthropic client"""
        try:
            import anthropic

            anthropic_config = self.config.get("anthropic", {})
            api_key = anthropic_config.get("api_key", "")

            # Resolve environment variable
            if api_key.startswith("${") and api_key.endswith("}"):
                env_var = api_key[2:-1]
                api_key = os.environ.get(env_var, "")

            if not api_key:
                api_key = os.environ.get("ANTHROPIC_API_KEY", "")

            self._client = anthropic.AsyncAnthropic(api_key=api_key)
            self._model = anthropic_config.get("model", "claude-sonnet-4-20250514")
            self._temperature = anthropic_config.get("temperature", 0.1)
            self._max_tokens = anthropic_config.get("max_tokens", 1024)

            logger.info(f"Anthropic client initialized with model: {self._model}")

        except ImportError:
            logger.error("Anthropic package not installed. Run: pip install anthropic")
            raise

    async def process_command(
        self,
        command: str,
        system_prompt: str,
        tools: List[Dict[str, Any]],
        use_cache: bool = True
    ) -> Dict[str, Any]:
        """Process a command through the LLM

        Args:
            command: User command text
            system_prompt: System prompt with context
            tools: Tool definitions
            use_cache: Whether to use caching

        Returns:
            Dict containing function call information
        """
        # Check cache
        if use_cache and self._cache_enabled:
            cached = self._cache.get(command)
            if cached:
                logger.debug(f"Cache hit for command: {command}")
                return cached

        # Rate limiting
        await self._wait_for_rate_limit()

        # Make API call with retries
        for attempt in range(self._max_retries):
            try:
                start_time = time.time()

                if self.provider == "openai":
                    result = await self._call_openai(command, system_prompt, tools)
                else:
                    result = await self._call_anthropic(command, system_prompt, tools)

                # Update statistics
                latency = (time.time() - start_time) * 1000
                self.stats.total_requests += 1
                self.stats.total_latency_ms += latency

                # Cache result
                if use_cache and self._cache_enabled and result.get("success"):
                    self._cache.set(command, result)

                return result

            except Exception as e:
                logger.warning(f"LLM API call failed (attempt {attempt + 1}): {e}")
                if attempt < self._max_retries - 1:
                    await asyncio.sleep(self._retry_delay)
                else:
                    return {
                        "success": False,
                        "error": str(e),
                        "error_code": "LLM_ERROR"
                    }

        return {
            "success": False,
            "error": "Max retries exceeded",
            "error_code": "LLM_ERROR"
        }

    async def _call_openai(
        self,
        command: str,
        system_prompt: str,
        tools: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Make OpenAI API call"""
        response = await self._client.chat.completions.create(
            model=self._model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            tools=tools,
            tool_choice="auto",
            temperature=self._temperature,
            max_tokens=self._max_tokens
        )

        # Update token stats
        if hasattr(response, 'usage') and response.usage:
            self.stats.total_prompt_tokens += response.usage.prompt_tokens
            self.stats.total_completion_tokens += response.usage.completion_tokens

        # Parse response
        message = response.choices[0].message

        if message.tool_calls:
            tool_call = message.tool_calls[0]
            import json
            return {
                "success": True,
                "function_name": tool_call.function.name,
                "parameters": json.loads(tool_call.function.arguments),
                "message": message.content or ""
            }
        else:
            return {
                "success": False,
                "error": "No function call in response",
                "error_code": "NO_FUNCTION_CALL",
                "message": message.content or ""
            }

    async def _call_anthropic(
        self,
        command: str,
        system_prompt: str,
        tools: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Make Anthropic API call"""
        # Convert tools to Anthropic format
        from .llm_tools import get_anthropic_tool_definitions
        anthropic_tools = get_anthropic_tool_definitions()

        response = await self._client.messages.create(
            model=self._model,
            max_tokens=self._max_tokens,
            system=system_prompt,
            tools=anthropic_tools,
            messages=[
                {"role": "user", "content": command}
            ]
        )

        # Update token stats
        if hasattr(response, 'usage') and response.usage:
            self.stats.total_prompt_tokens += response.usage.input_tokens
            self.stats.total_completion_tokens += response.usage.output_tokens

        # Parse response
        for block in response.content:
            if block.type == "tool_use":
                return {
                    "success": True,
                    "function_name": block.name,
                    "parameters": block.input,
                    "message": ""
                }

        # No tool call, get text response
        text_content = ""
        for block in response.content:
            if block.type == "text":
                text_content = block.text
                break

        return {
            "success": False,
            "error": "No function call in response",
            "error_code": "NO_FUNCTION_CALL",
            "message": text_content
        }

    async def _wait_for_rate_limit(self):
        """Wait for rate limit"""
        elapsed = time.time() - self._last_call_time
        if elapsed < self._min_interval:
            await asyncio.sleep(self._min_interval - elapsed)
        self._last_call_time = time.time()

    def get_stats(self) -> Dict[str, Any]:
        """Get LLM statistics"""
        return {
            "total_requests": self.stats.total_requests,
            "total_prompt_tokens": self.stats.total_prompt_tokens,
            "total_completion_tokens": self.stats.total_completion_tokens,
            "estimated_cost_usd": self.stats.estimated_cost_usd,
            "average_latency_ms": self.stats.average_latency_ms
        }

    def clear_cache(self):
        """Clear the command cache"""
        self._cache.clear()

    def get_cache_stats(self) -> Dict[str, Any]:
        """Get cache statistics"""
        return self._cache.get_stats()
