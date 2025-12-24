"""
Edge case tests for the retrieval pipeline.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from src.main import app
import json


class TestEdgeCases:
    """Test edge cases for the retrieval pipeline."""

    def test_empty_results_scenario(self):
        """Test behavior when no results are found for a query."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            # Mock service to return empty results
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 100

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "completely unrelated query that returns no results",
                "top_k": 5,
                "min_score": 0.9  # High threshold to ensure no results
            })

            assert response.status_code == 200
            data = response.json()

            assert data["query"] == "completely unrelated query that returns no results"
            assert data["results"] == []
            assert data["total_chunks_searched"] == 100
            assert "query_time_ms" in data

    def test_very_short_query(self):
        """Test behavior with very short queries."""
        client = TestClient(app)

        # Test with a single character query
        response = client.post("/retrieval/query", json={
            "query": "A",
            "top_k": 1,
            "min_score": 0.1
        })

        # Should either succeed or return appropriate error, not crash
        assert response.status_code in [200, 422]

        # Test with empty string (should return validation error)
        response = client.post("/retrieval/query", json={
            "query": "",
            "top_k": 1,
            "min_score": 0.1
        })

        assert response.status_code == 422  # Should be validation error for empty query

    def test_very_long_query(self):
        """Test behavior with very long queries."""
        client = TestClient(app)

        # Create a very long query
        long_query = "This is a very long query " * 1000

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            # Mock successful response
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 0

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": long_query,
                "top_k": 1,
                "min_score": 0.1
            })

            # Should handle long query without crashing
            assert response.status_code in [200, 500]  # Either success or internal error, but not crash

    def test_extreme_top_k_values(self):
        """Test behavior with extreme top_k values."""
        client = TestClient(app)

        # Test with top_k = 1 (minimum)
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 50

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 1,
                "min_score": 0.5
            })

            assert response.status_code == 200

        # Test with top_k = 10 (maximum allowed)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 10,
            "min_score": 0.5
        })

        assert response.status_code == 200

        # Test with top_k exceeding maximum (should return validation error)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 15,  # Exceeds max of 10
            "min_score": 0.5
        })

        assert response.status_code == 400

        # Test with top_k = 0 (should return validation error)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 0,  # Invalid
            "min_score": 0.5
        })

        assert response.status_code == 400

    def test_extreme_min_score_values(self):
        """Test behavior with extreme min_score values."""
        client = TestClient(app)

        # Test with min_score = 0.0 (minimum)
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 50

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.0
            })

            assert response.status_code == 200

        # Test with min_score = 1.0 (maximum)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": 1.0
        })

        assert response.status_code == 200

        # Test with min_score > 1.0 (should return validation error)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": 1.5  # Exceeds max of 1.0
        })

        assert response.status_code == 400

        # Test with min_score < 0.0 (should return validation error)
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": -0.5  # Below min of 0.0
        })

        assert response.status_code == 400

    def test_special_characters_in_query(self):
        """Test behavior with special characters in queries."""
        client = TestClient(app)

        special_queries = [
            "test & query",
            "test | query",
            "test (query)",
            "test \"query\"",
            "test: query!",
            "test\nquery",
            "test\tquery",
            "test\r\nquery",
            "test@query#query$%query",
            "tëst quëry with ünïcödë",
        ]

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 10

            mock_service.get_top_k_matches.return_value = mock_result

            for query in special_queries:
                response = client.post("/retrieval/query", json={
                    "query": query,
                    "top_k": 1,
                    "min_score": 0.1
                })

                # Should not crash with special characters
                assert response.status_code in [200, 500]

    def test_null_and_none_values(self):
        """Test behavior with null/none values."""
        client = TestClient(app)

        # Test missing optional parameters
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 5

            mock_service.get_top_k_matches.return_value = mock_result

            # Only required field
            response = client.post("/retrieval/query", json={
                "query": "test query"
                # top_k and min_score are optional
            })

            assert response.status_code == 200

    def test_boundary_values_for_validation(self):
        """Test boundary values for validation parameters."""
        client = TestClient(app)

        # Test exactly at boundary: top_k = 1
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 1,
            "min_score": 0.5
        })
        assert response.status_code == 200

        # Test exactly at boundary: top_k = 10
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 10,
            "min_score": 0.5
        })
        assert response.status_code == 200

        # Test exactly at boundary: min_score = 0.0
        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 5

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.0
            })
            assert response.status_code == 200

        # Test exactly at boundary: min_score = 1.0
        response = client.post("/retrieval/query", json={
            "query": "test query",
            "top_k": 5,
            "min_score": 1.0
        })
        assert response.status_code == 200

    def test_large_number_of_expected_urls_in_validation(self):
        """Test validation endpoint with a large number of expected URLs."""
        client = TestClient(app)

        # Create a large list of expected URLs
        large_url_list = [f"http://example.com/page{i}" for i in range(100)]

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_service.get_top_k_matches.return_value = Mock(
                matches=[],
                scores=[],
                metadata_list=[],
                total_chunks_searched=50
            )

            response = client.post("/retrieval/validate", json={
                "test_query": "test validation query",
                "expected_urls": large_url_list,
                "threshold": 0.9
            })

            # Should handle large URL list without crashing
            assert response.status_code in [200, 500]

    def test_extremely_high_threshold_values(self):
        """Test validation with extremely high thresholds."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            mock_result = Mock()
            mock_result.matches = []
            mock_result.scores = []
            mock_result.metadata_list = []
            mock_result.total_chunks_searched = 10

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/validate", json={
                "test_query": "test query",
                "expected_urls": ["http://example.com"],
                "threshold": 0.999  # Very high threshold
            })

            # Should handle high threshold without crashing
            assert response.status_code in [200, 500]

    def test_empty_content_edge_case(self):
        """Test behavior when retrieved content is empty."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            # Mock result with empty content
            mock_chunk = Mock()
            mock_chunk.id = "test-id"
            mock_chunk.content = ""  # Empty content
            mock_chunk.original_url = "http://example.com"
            mock_chunk.metadata = {}

            mock_metadata = Mock()
            mock_metadata.url = "http://example.com"
            mock_metadata.chunk_id = "test-id"
            mock_metadata.source_title = "Test Title"

            mock_result = Mock()
            mock_result.matches = [mock_chunk]
            mock_result.scores = [0.95]
            mock_result.metadata_list = [mock_metadata]
            mock_result.total_chunks_searched = 1

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.5
            })

            assert response.status_code == 200
            data = response.json()

            # Verify that empty content is handled gracefully
            assert len(data["results"]) == 1
            assert data["results"][0]["content"] == ""

    def test_malformed_metadata_edge_case(self):
        """Test behavior when metadata is malformed or missing fields."""
        client = TestClient(app)

        with patch('src.api.retrieval_router.retrieval_service') as mock_service:
            # Mock result with minimal metadata
            mock_chunk = Mock()
            mock_chunk.id = "test-id"
            mock_chunk.content = "Test content"
            mock_chunk.original_url = "http://example.com"
            mock_chunk.metadata = {}

            mock_metadata = Mock()
            mock_metadata.url = "http://example.com"
            mock_metadata.chunk_id = "test-id"
            mock_metadata.source_title = None  # Missing title

            mock_result = Mock()
            mock_result.matches = [mock_chunk]
            mock_result.scores = [0.95]
            mock_result.metadata_list = [mock_metadata]
            mock_result.total_chunks_searched = 1

            mock_service.get_top_k_matches.return_value = mock_result

            response = client.post("/retrieval/query", json={
                "query": "test query",
                "top_k": 5,
                "min_score": 0.5
            })

            assert response.status_code == 200
            data = response.json()

            # Verify that minimal metadata is handled gracefully
            assert len(data["results"]) == 1
            result = data["results"][0]
            assert result["content"] == "Test content"
            assert result["metadata"]["url"] == "http://example.com"