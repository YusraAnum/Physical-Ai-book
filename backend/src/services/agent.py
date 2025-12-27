from typing import List, Optional
from openai import OpenAI, APIError, APITimeoutError, RateLimitError
from ..models.chunk import RetrievedChunk
from ..config.settings import settings
import logging
import time

logger = logging.getLogger(__name__)

# Retry configuration
MAX_RETRIES = 3
INITIAL_BACKOFF = 1.0  # seconds


class AgentService:
    """OpenAI Agent orchestration service for generating responses based on retrieved context"""

    def __init__(self):
        # Add timeout to prevent hanging requests (30 seconds)
        self.client = OpenAI(
            api_key=settings.openai_api_key,
            timeout=30.0  # 30 second timeout
        )
        self.model = settings.openai_model

    def _call_openai_with_retry(self, make_request_fn):
        """
        Wrapper to call OpenAI with exponential backoff retry logic.

        Args:
            make_request_fn: Function that makes the API call

        Returns:
            Response from the API

        Raises:
            Exception: After all retries are exhausted
        """
        last_exception = None
        for attempt in range(MAX_RETRIES):
            try:
                return make_request_fn()
            except APITimeoutError as e:
                last_exception = e
                wait_time = INITIAL_BACKOFF * (2 ** attempt)
                logger.warning(f"OpenAI timeout (attempt {attempt + 1}/{MAX_RETRIES}), retrying in {wait_time}s...")
                time.sleep(wait_time)
            except RateLimitError as e:
                last_exception = e
                wait_time = INITIAL_BACKOFF * (2 ** attempt)
                logger.warning(f"OpenAI rate limit (attempt {attempt + 1}/{MAX_RETRIES}), retrying in {wait_time}s...")
                time.sleep(wait_time)
            except APIError as e:
                last_exception = e
                # For 5xx errors, retry; for 4xx (except rate limits), don't retry
                if hasattr(e, 'status_code') and e.status_code >= 500:
                    wait_time = INITIAL_BACKOFF * (2 ** attempt)
                    logger.warning(f"OpenAI API error {e.status_code} (attempt {attempt + 1}/{MAX_RETRIES}), retrying in {wait_time}s...")
                    time.sleep(wait_time)
                else:
                    raise  # Don't retry client errors

        # All retries exhausted
        if last_exception:
            raise last_exception
        raise Exception("OpenAI request failed after all retries")

    def generate_response(
        self,
        query: str,
        retrieved_chunks: List[RetrievedChunk],
        max_tokens: int = 500,
        context: Optional[str] = None
    ) -> str:
        """Generate a response based on the query and retrieved context chunks"""

        try:
            # 1️⃣ Format context
            context_text = self._format_context(retrieved_chunks)

            # 2️⃣ System prompt - instruct to answer based on the book's content
            system_prompt = (
                "You are a helpful book assistant answering questions about the book content. "
                "Answer the question using ONLY the provided context from the book. "
                "If the context contains related information, synthesize it to answer the question. "
                "If the context doesn't contain the answer, say so clearly. "
                "Always mention the relevant information from the context in your answer."
            )

            # 3️⃣ User message
            user_parts = []

            if context_text and context_text != "No relevant context found.":
                user_parts.append(f"Book Context:\n{context_text}")

            if context:
                user_parts.append(f"Additional Context:\n{context}")

            user_parts.append(f"Question: {query}")
            user_parts.append("Answer the question based on the Book Context above.")

            user_message = "\n\n".join(user_parts)

            # 4️⃣ OpenAI API call - with retry wrapper
            def make_request():
                return self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_message}
                    ],
                    max_tokens=max_tokens,
                    temperature=0.3
                )

            response = self._call_openai_with_retry(make_request)

            # 5️⃣ Extract and return the answer
            if response.choices and len(response.choices) > 0:
                answer = response.choices[0].message.content
                if answer:
                    return answer

            return "I could not generate an answer from the available context."

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            # Return a fallback answer instead of raising
            return f"Based on the retrieved context, I found relevant information but couldn't generate a complete answer. The context contains information about: {context_text[:200]}..." if context_text else "I found relevant sources but couldn't generate a response."

    def generate_empty_response(self, query: str) -> str:
        """Generate a response when no relevant chunks are found"""

        try:
            # Use retry wrapper for OpenAI call
            def make_request():
                return self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": "You are a book assistant. Answer clearly and concisely."},
                        {"role": "user", "content": f"I don't have enough information in the book to answer this question: {query}"}
                    ],
                    max_tokens=200,
                    temperature=0.3
                )

            response = self._call_openai_with_retry(make_request)

            if response.choices and len(response.choices) > 0:
                answer = response.choices[0].message.content
                if answer:
                    return answer

            return "I don't have enough information in the book to answer this question."

        except Exception as e:
            logger.error(f"Error generating empty response: {e}")
            # Return a simple fallback
            return "I don't have enough information in the book to answer this question."

    def _format_context(self, chunks: List[RetrievedChunk]) -> str:
        """Format the retrieved chunks into a context string"""

        if not chunks:
            return "No relevant context found."

        formatted_chunks = []
        for i, chunk in enumerate(chunks):
            formatted_chunks.append(
                f"Document {i + 1} (ID: {chunk.document_id}, Score: {chunk.similarity_score:.3f}):\n"
                f"{chunk.content}"
            )

        return "\n\n".join(formatted_chunks)
