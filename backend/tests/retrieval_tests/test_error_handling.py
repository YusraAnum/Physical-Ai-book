"""
Error handling tests for the retrieval service.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from src.main import app


class TestErrorHandling:
    """Test error handling functionality."""

    def test_query_endpoint_empty_query_error(self):
        """Test that empty query returns appropriate error."""
        client = TestClient(app)

        response = client.post("/retrieval/query", json={"query": ""})

        assert response.status_code == 422  # Unprocessable entity
        data = response.json()
        assert "detail" in data

    def test_query_endpoint_invalid_top_k_error(self):
        """Test that invalid top_k returns appropriate error."""
        client = TestClient(app)

        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": -1
        })

        assert response.status_code == 400  # Bad request
        data = response.json()
        assert "detail" in data
        assert "top_k must be between 1 and 10" in data["detail"]

    def test_query_endpoint_invalid_min_score_error(self):
        """Test that invalid min_score returns appropriate error."""
        client = TestClient(app)

        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": -0.5
        })

        assert response.status_code == 400  # Bad request
        data = response.json()
        assert "detail" in data
        assert "min_score must be between 0.0 and 1.0" in data["detail"]

    def test_query_endpoint_qdrant_unavailable(self):
        """Test error handling when Qdrant is unavailable."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_service.get_top_k_matches.side_effect = Exception("Qdrant connection failed")

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.5
            })

            assert response.status_code == 503  # Service unavailable
            data = response.json()
            assert "Qdrant database is currently unavailable" in data["detail"]

    def test_query_endpoint_cohere_unavailable(self):
        """Test error handling when Cohere API is unavailable."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_service.get_top_k_matches.side_effect = Exception("Cohere API error")

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.5
            })

            assert response.status_code == 503  # Service unavailable
            data = response.json()
            assert "Cohere API is currently unavailable" in data["detail"]

    def test_query_endpoint_internal_error(self):
        """Test error handling for internal errors."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_service.get_top_k_matches.side_effect = Exception("Unexpected error")

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.5
            })

            assert response.status_code == 500  # Internal server error
            data = response.json()
            assert "An internal error occurred during retrieval" in data["detail"]

    def test_validate_endpoint_empty_query_error(self):
        """Test that empty validation query returns appropriate error."""
        client = TestClient(app)

        response = client.post("/retrieval/validate", json={
            "test_query": "",
            "expected_urls": ["http://example.com"]
        })

        assert response.status_code == 422  # Unprocessable entity
        data = response.json()
        assert "detail" in data

    def test_validate_endpoint_empty_expected_urls_error(self):
        """Test that empty expected URLs returns appropriate error."""
        client = TestClient(app)

        response = client.post("/retrieval/validate", json={
            "test_query": "test query",
            "expected_urls": []
        })

        assert response.status_code == 422  # Unprocessable entity
        data = response.json()
        assert "detail" in data

    def test_validate_endpoint_invalid_threshold_error(self):
        """Test that invalid threshold returns appropriate error."""
        client = TestClient(app)

        response = client.post("/retrieval/validate", json={
            "test_query": "test query",
            "expected_urls": ["http://example.com"],
            "threshold": -0.5
        })

        assert response.status_code == 400  # Bad request
        data = response.json()
        assert "detail" in data
        assert "Threshold must be between 0.0 and 1.0" in data["detail"]

    def test_validate_full_endpoint_empty_query_error(self):
        """Test that empty validation query returns appropriate error for full validation."""
        client = TestClient(app)

        response = client.post("/retrieval/validate-full", json={
            "test_query": "",
            "expected_urls": ["http://example.com"]
        })

        assert response.status_code == 422  # Unprocessable entity
        data = response.json()
        assert "detail" in data

    def test_validate_full_endpoint_internal_error(self):
        """Test error handling for internal errors in full validation."""
        client = TestClient(app)

        with patch('src.services.end_to_end_retrieval_service.EndToEndRetrievalService') as mock_e2e_service_class:
            mock_instance = Mock()
            mock_instance.execute_pipeline_with_validation.side_effect = Exception("Validation failed")
            mock_e2e_service_class.return_value = mock_instance

            response = client.post("/retrieval/validate-full", json={
                "test_query": "test query",
                "expected_urls": ["http://example.com"],
                "threshold": 0.9
            })

            assert response.status_code == 500  # Internal server error
            data = response.json()
            assert "Full pipeline validation failed" in data["detail"]