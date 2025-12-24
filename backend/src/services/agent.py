from typing import List, Optional
from openai import OpenAI
from ..models.chunk import RetrievedChunk
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)


class AgentService:
    """OpenAI Agent orchestration service for generating responses based on retrieved context"""

    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.openai_model

    def generate_response(self, query: str, retrieved_chunks: List[RetrievedChunk],
                         max_tokens: int = 1000, context: Optional[str] = None) -> str:
        """Generate a response based on the query and retrieved context chunks"""
        try:
            # Format the context from retrieved chunks
            context_text = self._format_context(retrieved_chunks)

            # Create a system message that guides the agent's behavior
            system_message = (
                "You are a helpful assistant that answers questions based only on the provided context. "
                "If the context does not contain information to answer the question, clearly state that you don't have enough information. "
                "Always cite the source of information when possible and avoid making up information."
            )

            # Create the user message with the query and context
            user_message_parts = []
            if context_text and context_text != "No relevant context found.":
                user_message_parts.append(f"Retrieved Context:\n{context_text}")
            if context:
                user_message_parts.append(f"Additional Context:\n{context}")
            user_message_parts.append(f"Question: {query}")
            user_message_parts.append(
                "Please provide a detailed answer based on the context above. "
                "If the context does not contain relevant information, please state that clearly."
            )

            user_message = "\n\n".join(user_message_parts)

            # Call the OpenAI API to generate the response
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                max_tokens=max_tokens,
                temperature=0.3  # Lower temperature for more consistent, factual responses
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise

    def generate_empty_response(self, query: str) -> str:
        """Generate a response when no relevant chunks are found"""
        try:
            system_message = (
                "You are a helpful assistant. When asked a question, if you don't have enough information to answer, "
                "you should clearly state that you don't have enough information rather than making things up."
            )

            user_message = (
                f"Question: {query}\n\n"
                f"The system searched for relevant information but couldn't find any relevant content to answer this question. "
                f"Please acknowledge that you don't have enough information to answer this question based on the provided sources."
            )

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                max_tokens=300,
                temperature=0.3
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating empty response: {e}")
            raise

    def _format_context(self, chunks: List[RetrievedChunk]) -> str:
        """Format the retrieved chunks into a context string"""
        if not chunks:
            return "No relevant context found."

        formatted_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_text = f"Document {i+1} (ID: {chunk.document_id}, Score: {chunk.similarity_score:.3f}):\n{chunk.content}"
            formatted_chunks.append(chunk_text)

        return "\n\n".join(formatted_chunks)