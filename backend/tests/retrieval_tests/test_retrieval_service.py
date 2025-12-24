"""
Unit tests for the RetrievalService class.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.retrieval_service import RetrievalService
from src.models.query import Query


class TestRetrievalService:
    """Unit tests for RetrievalService."""

    def test_query_to_embedding_with_valid_text(self, mock_cohere_client):
        """Test converting a valid query text to embedding."""
        from src.utils.embedding_utils import normalize_vector

        service = RetrievalService()
        service.cohere_client = mock_cohere_client

        # Setup mock
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test
        result = service.query_to_embedding("test query")

        # Assertions - The result should be the normalized version of the mock return value
        expected_normalized = normalize_vector([0.1, 0.2, 0.3])
        assert result == expected_normalized
        mock_cohere_client.embed_single_text.assert_called_once_with("test query", input_type="search_query")

    def test_query_to_embedding_with_empty_text_raises_error(self):
        """Test that empty query text raises an error."""
        service = RetrievalService()

        with pytest.raises(ValueError, match="Query text cannot be empty"):
            service.query_to_embedding("")

    def test_query_to_embedding_with_whitespace_text_raises_error(self):
        """Test that whitespace-only query text raises an error."""
        service = RetrievalService()

        with pytest.raises(ValueError, match="Query text cannot be empty"):
            service.query_to_embedding("   ")

    def test_search_similar_chunks_with_valid_embedding(self, mock_qdrant_client):
        """Test searching for similar chunks with valid embedding."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client

        # Setup mock
        mock_result = [
            {"id": "1", "score": 0.9, "payload": {"content": "test", "url": "test.com", "chunk_id": "1"}}
        ]
        mock_qdrant_client.search_vectors.return_value = mock_result

        # Test
        result = service.search_similar_chunks([0.1, 0.2, 0.3], top_k=5, min_score=0.5)

        # Assertions
        assert result == mock_result
        mock_qdrant_client.search_vectors.assert_called_once_with(
            query_vector=[0.1, 0.2, 0.3],
            top_k=5
        )

    def test_search_similar_chunks_with_empty_embedding_raises_error(self):
        """Test that empty embedding raises an error."""
        service = RetrievalService()

        with pytest.raises(ValueError, match="Query embedding cannot be empty"):
            service.search_similar_chunks([])

    def test_get_top_k_matches_with_valid_query(self, mock_qdrant_client, mock_cohere_client):
        """Test getting top-K matches with valid query."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mocks
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_client.search_vectors.return_value = [
            {"id": "1", "score": 0.9, "payload": {"content": "test content", "url": "test.com", "chunk_id": "1"}}
        ]

        # Test
        result = service.get_top_k_matches("test query", top_k=5, min_score=0.5)

        # Assertions
        assert isinstance(result.query, Query)
        assert result.query.query_text == "test query"
        assert len(result.matches) == 1
        assert result.scores[0] == 0.9
        assert len(result.metadata_list) == 1

    def test_get_top_k_matches_with_empty_query_raises_error(self):
        """Test that empty query raises an error."""
        service = RetrievalService()

        with pytest.raises(ValueError, match="Query text cannot be empty"):
            service.get_top_k_matches("", top_k=5, min_score=0.5)

    def test_validate_query_embedding_match_success(self, mock_qdrant_client, mock_cohere_client):
        """Test successful validation of query embedding match."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mocks
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_client.search_vectors.return_value = [
            {"id": "1", "score": 0.9, "payload": {"content": "test content", "url": "expected.com", "chunk_id": "1"}}
        ]

        # Test
        result = service.validate_query_embedding_match("test query", "expected.com", threshold=0.8)

        # Assertions
        assert result is True

    def test_validate_query_embedding_match_failure(self, mock_qdrant_client, mock_cohere_client):
        """Test failed validation of query embedding match."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mocks
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_client.search_vectors.return_value = [
            {"id": "1", "score": 0.4, "payload": {"content": "test content", "url": "different.com", "chunk_id": "1"}}
        ]

        # Test
        result = service.validate_query_embedding_match("test query", "expected.com", threshold=0.8)

        # Assertions
        assert result is False

    def test_check_health_success(self, mock_qdrant_client, mock_cohere_client):
        """Test health check returns correct status."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mocks
        mock_qdrant_client.check_health.return_value = True
        mock_cohere_client.check_health.return_value = True

        # Test
        result = service.check_health()

        # Assertions
        assert result == {
            "qdrant_connected": True,
            "cohere_connected": True,
            "service_healthy": True
        }