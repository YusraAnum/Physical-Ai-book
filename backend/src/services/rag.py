from typing import List, Optional
from ..models.chunk import RetrievedChunk
from ..models.response import AgentResponse, SourceChunk
from .retrieval import RetrievalService
from .agent import AgentService
from ..config.settings import settings
import uuid
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class RAGService:
    """RAG orchestration service that combines retrieval and agent services"""

    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.agent_service = AgentService()

    def query_full_content(self, query_text: str, user_id: Optional[str] = None, context: Optional[str] = None) -> AgentResponse:
        """Process a query against the full book content"""
        query_id = str(uuid.uuid4())

        try:
            # If context is provided, we might want to use it differently
            # For now, we'll proceed with the normal search but could be enhanced to use context
            retrieved_chunks = self.retrieval_service.search_chunks(
                query_text=query_text,
                top_k=5,
                document_filters=None  # No document filters for full content search
            )

            # Generate response based on retrieved chunks
            if retrieved_chunks:
                answer = self.agent_service.generate_response(query_text, retrieved_chunks, context=context)
                has_sources = True
                source_chunks = retrieved_chunks
            else:
                # No relevant chunks found, generate response acknowledging this
                answer = self.agent_service.generate_empty_response(query_text)
                has_sources = False
                source_chunks = []

            # Convert RetrievedChunk objects to SourceChunk for the response
            sources = [
                SourceChunk(
                    chunk_id=chunk.chunk_id,
                    content=chunk.content,
                    document_id=chunk.document_id,
                    similarity_score=chunk.similarity_score,
                    metadata=chunk.metadata
                )
                for chunk in source_chunks
            ]

            # Calculate confidence based on similarity scores if sources exist
            confidence = None
            if source_chunks:
                avg_similarity = sum(chunk.similarity_score for chunk in source_chunks) / len(source_chunks)
                confidence = avg_similarity

            return AgentResponse(
                response_id=str(uuid.uuid4()),
                query_id=query_id,
                answer=answer,
                source_chunks=source_chunks,
                confidence_level=confidence,
                timestamp=datetime.now(),
                has_sources=has_sources
            )

        except Exception as e:
            logger.error(f"Error processing full content query: {e}")
            raise

    def query_selected_text(self, query_text: str, document_ids: List[str],
                           user_id: Optional[str] = None, context: Optional[str] = None) -> AgentResponse:
        """Process a query against selected text only"""
        query_id = str(uuid.uuid4())

        try:
            # Validate document IDs before searching
            if not self.retrieval_service.validate_document_filters(document_ids):
                raise ValueError("Invalid document IDs provided for filtering")

            # Retrieve relevant chunks from the selected documents only
            retrieved_chunks = self.retrieval_service.search_chunks(
                query_text=query_text,
                top_k=5,
                document_filters=document_ids
            )

            # Generate response based on retrieved chunks
            if retrieved_chunks:
                answer = self.agent_service.generate_response(query_text, retrieved_chunks, context=context)
                has_sources = True
                source_chunks = retrieved_chunks
            else:
                # No relevant chunks found in selected documents, generate response acknowledging this
                answer = self.agent_service.generate_empty_response(query_text)
                has_sources = False
                source_chunks = []

            # Convert RetrievedChunk objects to SourceChunk for the response
            sources = [
                SourceChunk(
                    chunk_id=chunk.chunk_id,
                    content=chunk.content,
                    document_id=chunk.document_id,
                    similarity_score=chunk.similarity_score,
                    metadata=chunk.metadata
                )
                for chunk in source_chunks
            ]

            # Calculate confidence based on similarity scores if sources exist
            confidence = None
            if source_chunks:
                avg_similarity = sum(chunk.similarity_score for chunk in source_chunks) / len(source_chunks)
                confidence = avg_similarity

            return AgentResponse(
                response_id=str(uuid.uuid4()),
                query_id=query_id,
                answer=answer,
                source_chunks=source_chunks,
                confidence_level=confidence,
                timestamp=datetime.now(),
                has_sources=has_sources
            )

        except Exception as e:
            logger.error(f"Error processing selected text query: {e}")
            raise