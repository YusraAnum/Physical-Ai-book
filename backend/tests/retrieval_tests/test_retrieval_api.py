"""
Integration tests for the retrieval API endpoints.
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


class TestRetrievalAPI:
    """Integration tests for retrieval API endpoints."""

    def test_query_endpoint_success(self, client, mock_qdrant_client, mock_cohere_client):
        """Test successful query to the retrieval endpoint."""
        # Mock the retrieval service
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = [Mock()]
            mock_result.matches[0].id = "1"
            mock_result.matches[0].content = "test content"
            mock_result.scores = [0.9]
            mock_result.metadata_list = [Mock()]
            mock_result.metadata_list[0].url = "test.com"
            mock_result.metadata_list[0].chunk_id = "1"
            mock_result.metadata_list[0].source_title = "Test Title"
            mock_result.total_chunks_searched = 1

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.5
            })

            assert response.status_code == 200
            data = response.json()
            assert "results" in data
            assert len(data["results"]) == 1
            assert data["results"][0]["content"] == "test content"

    def test_query_endpoint_empty_query(self, client):
        """Test query endpoint with empty query."""
        response = client.post("/retrieval/query", json={
            "query": "",
            "top_k": 5,
            "min_score": 0.5
        })

        assert response.status_code == 422  # Unprocessable entity

    def test_query_endpoint_invalid_top_k(self, client):
        """Test query endpoint with invalid top_k value."""
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 15,  # Too high
            "min_score": 0.5
        })

        assert response.status_code == 400  # Bad request

    def test_query_endpoint_invalid_min_score(self, client):
        """Test query endpoint with invalid min_score value."""
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": 1.5  # Too high
        })

        assert response.status_code == 400  # Bad request

    def test_health_endpoint_success(self, client, mock_qdrant_client, mock_cohere_client):
        """Test successful health check."""
        # Mock the health service
        with patch('src.api.retrieval_router.health_service') as mock_health_service:
            mock_health_service.check_overall_health.return_value = {
                "status": "healthy",
                "overall_healthy": True,
                "components": {
                    "qdrant": {
                        "status": "healthy",
                        "connected": True
                    },
                    "cohere": {
                        "status": "healthy",
                        "connected": True
                    }
                }
            }

            response = client.get("/retrieval/health")

            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "healthy"
            assert data["qdrant_connected"] is True
            assert data["cohere_connected"] is True

    def test_health_endpoint_failure(self, client, mock_qdrant_client, mock_cohere_client):
        """Test health check with service failure."""
        # Mock the health service
        with patch('src.api.retrieval_router.health_service') as mock_health_service:
            mock_health_service.check_overall_health.return_value = {
                "status": "unhealthy",
                "overall_healthy": False,
                "components": {
                    "qdrant": {
                        "status": "unhealthy",
                        "connected": False
                    },
                    "cohere": {
                        "status": "healthy",
                        "connected": True
                    }
                }
            }

            response = client.get("/retrieval/health")

            assert response.status_code == 200  # Health endpoint returns 200 even if service is unhealthy
            data = response.json()
            assert data["status"] == "unhealthy"
            assert data["qdrant_connected"] is False
            assert data["cohere_connected"] is True

    def test_validate_endpoint_success(self, client, mock_qdrant_client, mock_cohere_client):
        """Test successful validation endpoint."""
        # Mock the retrieval service
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = [Mock()]
            mock_result.metadata_list = [Mock()]
            mock_result.metadata_list[0].url = "expected.com"
            mock_result.scores = [0.9]
            mock_result.total_chunks_searched = 1

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/validate", json={
                "test_query": "test query",
                "expected_urls": ["expected.com"],
                "threshold": 0.8
            })

            assert response.status_code == 200
            data = response.json()
            assert data["success"] is True
            assert data["accuracy"] == 1.0
            assert "expected.com" in data["retrieved_urls"]

    def test_validate_endpoint_empty_query(self, client):
        """Test validation endpoint with empty query."""
        response = client.post("/retrieval/validate", json={
            "test_query": "",
            "expected_urls": ["expected.com"],
            "threshold": 0.8
        })

        assert response.status_code == 422  # Unprocessable entity

    def test_validate_endpoint_empty_expected_urls(self, client):
        """Test validation endpoint with empty expected URLs."""
        response = client.post("/retrieval/validate", json={
            "test_query": "test query",
            "expected_urls": [],
            "threshold": 0.8
        })

        assert response.status_code == 422  # Unprocessable entity