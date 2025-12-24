"""
Test for the specific "ROS 2 middleware concepts" query to verify relevant results.
"""
import pytest
from unittest.mock import Mock, patch
from src.services.retrieval_service import RetrievalService


class TestROSMiddlewareQuery:
    """Test for the specific ROS 2 middleware query."""

    def test_ros_middleware_query_returns_relevant_results(self, mock_qdrant_client, mock_cohere_client):
        """Test that querying for 'ROS 2 middleware concepts' returns relevant results."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mock to return results related to ROS 2 middleware
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.92,
                "payload": {
                    "content": "ROS 2 uses a DDS-based middleware that provides reliable communication between nodes.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-intro-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            },
            {
                "id": "chunk-2",
                "score": 0.87,
                "payload": {
                    "content": "The middleware layer in ROS 2 handles communication protocols and data serialization.",
                    "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
                    "chunk_id": "middleware-impl-002",
                    "source_title": "Chapter 2: Communication Primitives"
                }
            }
        ]

        # Setup mock embedding
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test the query
        result = service.get_top_k_matches("ROS 2 middleware concepts", top_k=5, min_score=0.5)

        # Verify the results are relevant to ROS 2 middleware
        assert len(result.matches) > 0
        assert result.matches[0].content is not None
        assert "ROS 2" in result.matches[0].content or "middleware" in result.matches[0].content.lower()

        # Verify scores are in descending order
        for i in range(len(result.scores) - 1):
            assert result.scores[i] >= result.scores[i + 1]

        # Verify metadata is present
        assert len(result.metadata_list) == len(result.matches)
        assert result.metadata_list[0].url is not None
        assert result.metadata_list[0].chunk_id is not None

    def test_ros_middleware_query_with_different_variations(self, mock_qdrant_client, mock_cohere_client):
        """Test that variations of the ROS 2 middleware query return relevant results."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.89,
                "payload": {
                    "content": "The middleware in ROS 2 provides communication between distributed nodes.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-intro-001"
                }
            }
        ]

        # Setup mock embedding
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test different query variations
        queries = [
            "Explain ROS 2 middleware",
            "What is ROS 2 middleware?",
            "How does ROS 2 middleware work?",
            "ROS 2 communication middleware"
        ]

        for query in queries:
            result = service.get_top_k_matches(query, top_k=3, min_score=0.5)

            # Verify we get results for each query
            assert len(result.matches) > 0
            assert result.scores[0] >= 0.5  # Above minimum threshold

    def test_query_similarity_scoring(self, mock_qdrant_client, mock_cohere_client):
        """Test that similar queries return results with appropriate similarity scores."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.95,
                "payload": {
                    "content": "ROS 2 uses DDS middleware for node communication.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-intro-001"
                }
            }
        ]

        # Setup mock embedding
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test query that should have high relevance
        result = service.get_top_k_matches("Explain ROS 2 middleware concepts", top_k=1, min_score=0.5)

        # Verify we get a high relevance score for matching content
        assert len(result.scores) > 0
        assert result.scores[0] > 0.8  # High relevance expected

    def test_no_results_for_irrelevant_query(self, mock_qdrant_client, mock_cohere_client):
        """Test behavior when no relevant results are found."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mock to return no results
        mock_qdrant_client.search_vectors.return_value = []

        # Setup mock embedding
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test with a query that should have no matches
        result = service.get_top_k_matches("completely unrelated query", top_k=5, min_score=0.8)

        # Verify we get empty results
        assert len(result.matches) == 0
        assert len(result.scores) == 0
        assert len(result.metadata_list) == 0