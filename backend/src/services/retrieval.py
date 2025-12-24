from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from ..models.chunk import RetrievedChunk
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)


class RetrievalService:
    """Qdrant retrieval service for finding relevant document chunks based on user queries"""

    def __init__(self):
        # Initialize Qdrant client
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            # If no API key, assume it's a local instance
            self.client = QdrantClient(
                url=settings.qdrant_url,
                timeout=10
            )

        # Initialize OpenAI client for embedding generation
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

        self.collection_name = settings.collection_name

    def _generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a text using OpenAI's embedding API"""
        try:
            response = self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-ada-002"  # Using OpenAI's embedding model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def validate_document_filters(self, document_filters: Optional[List[str]]) -> bool:
        """Validate document filters to ensure they meet requirements"""
        if not document_filters:
            return True  # No filters is valid

        # Check that we have at least one filter
        if len(document_filters) == 0:
            return False

        # Check that each filter is a non-empty string
        for doc_filter in document_filters:
            if not doc_filter or not isinstance(doc_filter, str):
                return False

        # Optional: Check that the document IDs exist in the collection
        # For now, we'll just validate the format
        return True

    def search_chunks(self, query_text: str, top_k: int = 5, document_filters: Optional[List[str]] = None) -> List[RetrievedChunk]:
        """Search for relevant chunks in Qdrant based on the query"""
        try:
            # Validate document filters
            if document_filters is not None and not self.validate_document_filters(document_filters):
                raise ValueError("Invalid document filters provided")

            # Generate embedding for the query
            query_embedding = self._generate_embedding(query_text)

            # Prepare filters if document filters are provided
            filters = None
            if document_filters:
                filters = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchAny(any=document_filters)
                        )
                    ]
                )

            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=filters,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            # Convert search results to RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results:
                payload = result.payload
                retrieved_chunk = RetrievedChunk(
                    chunk_id=result.id,
                    content=payload.get('text', ''),
                    document_id=payload.get('document_id', ''),
                    metadata=payload.get('metadata', {}),
                    similarity_score=result.score
                )
                retrieved_chunks.append(retrieved_chunk)

            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error during chunk search: {e}")
            raise

    def check_collection_exists(self) -> bool:
        """Check if the collection exists in Qdrant"""
        try:
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]
            return self.collection_name in collection_names
        except Exception as e:
            logger.error(f"Error checking collection existence: {e}")
            return False