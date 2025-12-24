"""
Retrieval service for querying the Qdrant vector database and retrieving relevant text chunks.
"""
from typing import List, Optional, Dict, Any
import time
from ..models.query import Query
from ..models.text_chunk import TextChunk
from ..models.metadata import Metadata
from ..models.retrieval_result import RetrievalResult
from ..models.vector_embedding import VectorEmbedding
from .qdrant_client_wrapper import QdrantClientWrapper
from .cohere_client_wrapper import CohereClientWrapper
from .content_verification_service import ContentVerificationService
from ..utils.embedding_utils import create_vector_embedding, normalize_vector


class RetrievalService:
    """
    Service class for handling retrieval operations from the vector database.
    """
    def __init__(self):
        """
        Initialize the retrieval service with Qdrant and Cohere clients.
        """
        self.qdrant_client = QdrantClientWrapper()
        self.cohere_client = CohereClientWrapper()
        self.content_verification_service = ContentVerificationService()

    def query_to_embedding(self, query_text: str) -> List[float]:
        """
        Convert a text query to a vector embedding using Cohere API.

        Args:
            query_text: The text query to convert

        Returns:
            Vector embedding of the query text
        """
        if not query_text or len(query_text.strip()) == 0:
            raise ValueError("Query text cannot be empty")

        try:
            embedding = self.cohere_client.embed_single_text(query_text, input_type="search_query")
            return normalize_vector(embedding)
        except Exception as e:
            raise Exception(f"Failed to generate embedding for query: {str(e)}")

    def search_similar_chunks_with_error_handling(self, query_embedding: List[float], top_k: int = 5, min_score: float = 0.5) -> List[Dict[str, Any]]:
        """
        Search for similar text chunks in the Qdrant database with error handling.

        Args:
            query_embedding: The query vector embedding
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold

        Returns:
            List of similar chunks with scores and metadata
        """
        if not query_embedding:
            raise ValueError("Query embedding cannot be empty")

        try:
            # Search in Qdrant
            search_results = self.qdrant_client.search_vectors(
                query_vector=query_embedding,
                top_k=top_k
            )

            # Filter results based on minimum score and ensure complete metadata
            filtered_results = []
            for result in search_results:
                if result["score"] >= min_score:
                    # Ensure all required metadata fields are present
                    payload = result["payload"]
                    if "url" in payload and "chunk_id" in payload:
                        filtered_results.append(result)

            return filtered_results
        except Exception as e:
            # Log the error and re-raise with context
            print(f"Error searching for similar chunks: {str(e)}")
            raise Exception(f"Failed to search for similar chunks: {str(e)}")

    def search_similar_chunks(self, query_embedding: List[float], top_k: int = 5, min_score: float = 0.5) -> List[Dict[str, Any]]:
        """
        Search for similar text chunks in the Qdrant database.

        Args:
            query_embedding: The query vector embedding
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold

        Returns:
            List of similar chunks with scores and metadata
        """
        return self.search_similar_chunks_with_error_handling(query_embedding, top_k, min_score)

    def get_top_k_matches(self, query_text: str, top_k: int = 5, min_score: float = 0.5) -> RetrievalResult:
        """
        Get the top-K most similar text chunks for a query.

        Args:
            query_text: The text query
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold

        Returns:
            RetrievalResult containing matches, scores, and metadata
        """
        if not query_text or len(query_text.strip()) == 0:
            raise ValueError("Query text cannot be empty")

        start_time = time.time()

        try:
            # Create query object
            query = Query(query_text=query_text)

            # Convert query to embedding
            query_embedding = self.query_to_embedding(query_text)

            # Search for similar chunks
            search_results = self.search_similar_chunks(query_embedding, top_k, min_score)

            # Extract matches, scores, and metadata
            matches = []
            scores = []
            metadata_list = []

            for result in search_results:
                payload = result["payload"]

                # Create TextChunk from payload
                text_chunk = TextChunk(
                    id=result["id"],
                    content=payload.get("content", ""),
                    original_url=payload.get("url", ""),
                    metadata=payload
                )

                # Create Metadata from payload
                metadata = Metadata(
                    url=payload.get("url", ""),
                    chunk_id=payload.get("chunk_id", ""),
                    source_title=payload.get("source_title")
                )

                matches.append(text_chunk)
                scores.append(result["score"])
                metadata_list.append(metadata)

            retrieval_time = time.time() - start_time

            # Create retrieval result
            retrieval_result = RetrievalResult(
                query=query,
                matches=matches,
                scores=scores,
                metadata_list=metadata_list,
                retrieval_time=retrieval_time,
                total_chunks_searched=len(search_results)
            )

            return retrieval_result
        except Exception as e:
            # Log the error and re-raise with context
            print(f"Error in get_top_k_matches: {str(e)}")
            raise Exception(f"Failed to retrieve top-K matches: {str(e)}")

    def get_top_k_matches_with_detailed_error_handling(self, query_text: str, top_k: int = 5, min_score: float = 0.5) -> RetrievalResult:
        """
        Get the top-K most similar text chunks for a query with detailed error handling.

        Args:
            query_text: The text query
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold

        Returns:
            RetrievalResult containing matches, scores, and metadata
        """
        if not query_text or len(query_text.strip()) == 0:
            raise ValueError("Query text cannot be empty")

        start_time = time.time()

        try:
            # Create query object
            query = Query(query_text=query_text)

            # Convert query to embedding with error handling
            try:
                query_embedding = self.query_to_embedding(query_text)
            except Exception as e:
                raise Exception(f"Embedding generation failed: {str(e)}")

            # Search for similar chunks with error handling
            try:
                search_results = self.search_similar_chunks(query_embedding, top_k, min_score)
            except Exception as e:
                raise Exception(f"Vector search failed: {str(e)}")

            # Extract matches, scores, and metadata
            matches = []
            scores = []
            metadata_list = []

            for result in search_results:
                payload = result["payload"]

                # Create TextChunk from payload
                text_chunk = TextChunk(
                    id=result["id"],
                    content=payload.get("content", ""),
                    original_url=payload.get("url", ""),
                    metadata=payload
                )

                # Create Metadata from payload
                metadata = Metadata(
                    url=payload.get("url", ""),
                    chunk_id=payload.get("chunk_id", ""),
                    source_title=payload.get("source_title")
                )

                matches.append(text_chunk)
                scores.append(result["score"])
                metadata_list.append(metadata)

            retrieval_time = time.time() - start_time

            # Create retrieval result
            retrieval_result = RetrievalResult(
                query=query,
                matches=matches,
                scores=scores,
                metadata_list=metadata_list,
                retrieval_time=retrieval_time,
                total_chunks_searched=len(search_results)
            )

            return retrieval_result
        except Exception as e:
            # Log the error with more context
            error_context = {
                "query_length": len(query_text),
                "top_k": top_k,
                "min_score": min_score,
                "timestamp": time.time()
            }
            print(f"Error in get_top_k_matches_with_detailed_error_handling: {str(e)}, Context: {error_context}")
            raise

    def validate_query_embedding_match(self, query_text: str, expected_url: str, threshold: float = 0.8) -> bool:
        """
        Validate that a query returns expected content.

        Args:
            query_text: The query text to test
            expected_url: The URL that should be in the results
            threshold: Minimum score threshold for validation

        Returns:
            True if validation passes, False otherwise
        """
        result = self.get_top_k_matches(query_text, top_k=1, min_score=0.0)

        if not result.matches:
            return False

        # Check if the expected URL is in the results
        for metadata in result.metadata_list:
            if metadata.url == expected_url and result.scores[0] >= threshold:
                return True

        return False

    def add_timing_measurements(self, retrieval_result: 'RetrievalResult', query_time: float) -> 'RetrievalResult':
        """
        Add timing measurements to the retrieval result.

        Args:
            retrieval_result: The retrieval result to add timing to
            query_time: Time taken for the query in seconds

        Returns:
            The retrieval result with added timing information
        """
        # This method would add additional timing information to the result
        # For now, we'll just return the result as is since timing is already included
        return retrieval_result

    def check_health(self) -> Dict[str, bool]:
        """
        Check the health of the retrieval service components.

        Returns:
            Dictionary with health status of each component
        """
        qdrant_healthy = self.qdrant_client.check_health()
        cohere_healthy = self.cohere_client.check_health()

        return {
            "qdrant_connected": qdrant_healthy,
            "cohere_connected": cohere_healthy,
            "service_healthy": qdrant_healthy and cohere_healthy
        }