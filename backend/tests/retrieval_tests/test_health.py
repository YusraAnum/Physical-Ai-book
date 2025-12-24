"""
Health check tests for the retrieval service.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from src.main import app
from src.services.health_service import HealthService


class TestHealthCheck:
    """Test health check functionality."""

    def test_health_check_endpoint(self):
        """Test the health check endpoint returns proper response."""
        client = TestClient(app)

        # Mock the health service to return healthy status
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

            # Verify the response
            assert response.status_code == 200
            data = response.json()

            assert data["status"] == "healthy"
            assert data["qdrant_connected"] is True
            assert data["cohere_connected"] is True
            assert "last_heartbeat" in data

    def test_health_check_with_unhealthy_components(self):
        """Test the health check endpoint when components are unhealthy."""
        client = TestClient(app)

        # Mock the health service to return unhealthy status
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

            # Verify the response
            assert response.status_code == 200  # Health check itself succeeds
            data = response.json()

            assert data["status"] == "unhealthy"
            assert data["qdrant_connected"] is False
            assert data["cohere_connected"] is True
            assert "last_heartbeat" in data

    def test_health_check_internal_error(self):
        """Test the health check endpoint when an internal error occurs."""
        client = TestClient(app)

        # Mock the health service to raise an exception
        with patch('src.api.retrieval_router.health_service') as mock_health_service:
            mock_health_service.check_overall_health.side_effect = Exception("Health check failed")

            response = client.get("/retrieval/health")

            # Verify the error response
            assert response.status_code == 500
            data = response.json()

            assert "detail" in data
            assert "Health check failed" in data["detail"]

    def test_health_service_functionality(self):
        """Test the health service directly."""
        # Mock the client wrappers
        with patch('src.services.health_service.QdrantClientWrapper') as mock_qdrant, \
             patch('src.services.health_service.CohereClientWrapper') as mock_cohere:

            # Set up mock return values
            mock_qdrant.return_value.check_health.return_value = True
            mock_cohere.return_value.check_health.return_value = True

            health_service = HealthService()
            result = health_service.check_overall_health()

            # Verify the result
            assert result["overall_healthy"] is True
            assert result["components"]["qdrant"]["connected"] is True
            assert result["components"]["cohere"]["connected"] is True
            assert result["status"] == "healthy"

    def test_health_service_with_unhealthy_components(self):
        """Test the health service with unhealthy components."""
        # Mock the client wrappers
        with patch('src.services.health_service.QdrantClientWrapper') as mock_qdrant, \
             patch('src.services.health_service.CohereClientWrapper') as mock_cohere:

            # Set up mock return values - qdrant is unhealthy
            mock_qdrant.return_value.check_health.return_value = False
            mock_cohere.return_value.check_health.return_value = True

            health_service = HealthService()
            result = health_service.check_overall_health()

            # Verify the result
            assert result["overall_healthy"] is False
            assert result["components"]["qdrant"]["connected"] is False
            assert result["components"]["cohere"]["connected"] is True
            assert result["status"] == "unhealthy"

    def test_health_service_exception_handling(self):
        """Test the health service exception handling."""
        # Test Qdrant health check exception
        with patch('src.services.health_service.QdrantClientWrapper') as mock_qdrant:
            mock_qdrant.return_value.check_health.side_effect = Exception("Qdrant unavailable")

            health_service = HealthService()
            result = health_service.check_qdrant_health()

            assert result is False

        # Test Cohere health check exception
        with patch('src.services.health_service.CohereClientWrapper') as mock_cohere:
            mock_cohere.return_value.check_health.side_effect = Exception("Cohere unavailable")

            health_service = HealthService()
            result = health_service.check_cohere_health()

            assert result is False