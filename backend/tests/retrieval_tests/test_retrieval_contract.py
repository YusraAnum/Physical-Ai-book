"""
Contract tests for the retrieval API endpoints based on the API contract.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from src.api.retrieval_router import router, retrieval_service
from src.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


class TestRetrievalContract:
    """Contract tests for retrieval API endpoints."""

    def test_retrieval_query_endpoint_contract(self, client, mock_qdrant_client, mock_cohere_client):
        """Test that the POST /retrieval/query endpoint follows the contract."""
        # Mock the retrieval service
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = [Mock(), Mock()]
            mock_result.matches[0].id = "chunk-123"
            mock_result.matches[0].content = "ROS 2 uses a DDS-based middleware that provides..."
            mock_result.matches[0].metadata = Mock()
            mock_result.matches[0].metadata.url = "http://localhost:3002/docs/chapter-1/middleware-concept"
            mock_result.matches[0].metadata.chunk_id = "middleware-intro-001"
            mock_result.matches[0].metadata.source_title = "Chapter 1: ROS 2 as Middleware"

            mock_result.matches[1].id = "chunk-456"
            mock_result.matches[1].content = "The middleware layer in ROS 2 handles communication..."
            mock_result.matches[1].metadata = Mock()
            mock_result.matches[1].metadata.url = "http://localhost:3002/docs/chapter-2/nodes-topics-services"
            mock_result.matches[1].metadata.chunk_id = "middleware-impl-002"
            mock_result.matches[1].metadata.source_title = "Chapter 2: Communication Primitives"

            mock_result.scores = [0.92, 0.87]
            mock_result.metadata_list = [mock_result.matches[0].metadata, mock_result.matches[1].metadata]
            mock_result.total_chunks_searched = 150

            mock_service.get_top_k_matches.return_value = mock_result

            # Test the endpoint with example request from contract
            response = client.post("/retrieval/query", json={
                "query": "Explain ROS 2 middleware concepts",
                "top_k": 3,
                "min_score": 0.6
            })

            # Validate response status
            assert response.status_code == 200

            # Validate response structure
            data = response.json()

            # Check required fields exist
            assert "query" in data
            assert "results" in data
            assert "query_time_ms" in data
            assert "total_chunks_searched" in data

            # Validate query field
            assert data["query"] == "Explain ROS 2 middleware concepts"

            # Validate results structure
            assert isinstance(data["results"], list)
            assert len(data["results"]) == 2  # Based on our mock

            # Validate first result structure
            first_result = data["results"][0]
            assert "id" in first_result
            assert "content" in first_result
            assert "score" in first_result
            assert "metadata" in first_result

            assert first_result["id"] == "chunk-123"
            assert "DDS-based middleware" in first_result["content"]
            assert first_result["score"] == 0.92

            # Validate metadata structure
            metadata = first_result["metadata"]
            assert "url" in metadata
            assert "chunk_id" in metadata
            assert "source_title" in metadata

            assert metadata["url"] == "http://localhost:3002/docs/chapter-1/middleware-concept"
            assert metadata["chunk_id"] == "middleware-intro-001"
            assert metadata["source_title"] == "Chapter 1: ROS 2 as Middleware"

            # Validate numeric fields
            assert isinstance(data["query_time_ms"], (int, float))
            assert isinstance(data["total_chunks_searched"], int)
            assert data["total_chunks_searched"] == 150

    def test_retrieval_query_endpoint_default_values(self, client, mock_qdrant_client, mock_cohere_client):
        """Test that the endpoint works with default values."""
        # Mock the retrieval service
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = [Mock()]
            mock_result.matches[0].id = "chunk-1"
            mock_result.matches[0].content = "test content"
            mock_result.matches[0].metadata = Mock()
            mock_result.matches[0].metadata.url = "test.com"
            mock_result.matches[0].metadata.chunk_id = "1"
            mock_result.matches[0].metadata.source_title = None
            mock_result.scores = [0.8]
            mock_result.metadata_list = [mock_result.matches[0].metadata]
            mock_result.total_chunks_searched = 1

            mock_service.get_top_k_matches.return_value = mock_result

            # Test with minimal request (only required field)
            response = client.post("/retrieval/query", json={
                "query": "test query"
            })

            assert response.status_code == 200
            data = response.json()

            # Should have defaults applied
            assert data["query"] == "test query"

    def test_retrieval_query_endpoint_error_responses(self, client):
        """Test error response format matches contract."""
        # Test with empty query (should return 422)
        response = client.post("/retrieval/query", json={
            "query": "",
            "top_k": 5,
            "min_score": 0.5
        })

        assert response.status_code == 422

        # Test with invalid top_k (should return 400)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 15,  # Too high
            "min_score": 0.5
        })

        assert response.status_code == 400

        # Test with invalid min_score (should return 400)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": 1.5  # Too high
        })

        assert response.status_code == 400

    def test_health_endpoint_contract(self, client, mock_qdrant_client, mock_cohere_client):
        """Test that the GET /retrieval/health endpoint follows the contract."""
        # Mock the retrieval service
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_service.check_health.return_value = {
                "qdrant_connected": True,
                "cohere_connected": True,
                "service_healthy": True
            }

            response = client.get("/retrieval/health")

            assert response.status_code == 200

            data = response.json()

            # Check required fields exist
            assert "status" in data
            assert "qdrant_connected" in data
            assert "cohere_connected" in data
            assert "last_heartbeat" in data

            # Validate field types
            assert isinstance(data["status"], str)
            assert isinstance(data["qdrant_connected"], bool)
            assert isinstance(data["cohere_connected"], bool)
            assert isinstance(data["last_heartbeat"], str)

            # Validate values
            assert data["status"] in ["healthy", "degraded", "unavailable"]
            assert data["qdrant_connected"] is True
            assert data["cohere_connected"] is True

    def test_validate_endpoint_contract(self, client, mock_qdrant_client, mock_cohere_client):
        """Test that the POST /retrieval/validate endpoint follows the contract."""
        # Mock the retrieval service
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = [Mock()]
            mock_result.metadata_list = [Mock()]
            mock_result.metadata_list[0].url = "http://localhost:3002/docs/chapter-1/middleware-concept"
            mock_result.scores = [0.95]
            mock_result.total_chunks_searched = 1

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/validate", json={
                "test_query": "test validation query",
                "expected_urls": ["http://localhost:3002/docs/chapter-1/middleware-concept"],
                "threshold": 0.9
            })

            assert response.status_code == 200

            data = response.json()

            # Check required fields exist
            assert "success" in data
            assert "accuracy" in data
            assert "retrieved_urls" in data
            assert "matches_expected" in data
            assert "details" in data

            # Validate field types
            assert isinstance(data["success"], bool)
            assert isinstance(data["accuracy"], (int, float))
            assert isinstance(data["retrieved_urls"], list)
            assert isinstance(data["matches_expected"], bool)
            assert isinstance(data["details"], dict)

            # Validate details structure
            details = data["details"]
            assert "top_match_accuracy" in details
            assert "content_similarity" in details
            assert "metadata_correctness" in details