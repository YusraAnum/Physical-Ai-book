"""
Performance tests to verify the 2-second completion requirement.
"""
import time
import pytest
from unittest.mock import Mock, patch
from src.services.end_to_end_retrieval_service import EndToEndRetrievalService


class TestPerformanceRequirements:
    """Performance tests to verify the 2-second completion requirement."""

    def test_end_to_end_pipeline_completes_within_2_seconds(self, mock_qdrant_client, mock_cohere_client):
        """Test that the end-to-end pipeline completes within 2 seconds as per requirements."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.9,
                "payload": {
                    "content": "ROS 2 uses a DDS-based middleware that provides reliable communication.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-intro-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Measure execution time
        start_time = time.time()
        result = service.execute_full_pipeline("ROS 2 middleware concepts", top_k=1, min_score=0.5)
        end_time = time.time()

        execution_time = end_time - start_time

        # Verify it completes within 2 seconds
        assert execution_time <= 2.0, f"Pipeline took {execution_time:.2f}s, which exceeds 2-second requirement"
        assert result["pipeline_time_seconds"] <= 2.0

        # Also verify the time is captured correctly in the result
        assert result["pipeline_time_seconds"] == pytest.approx(execution_time, abs=0.1)  # Allow small variance

    def test_pipeline_performance_validation_function(self, mock_qdrant_client, mock_cohere_client):
        """Test the explicit performance validation function."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.85,
                "payload": {
                    "content": "Content for performance testing.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "test-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Validate performance against 2-second requirement
        result = service.validate_pipeline_performance("performance test query", max_pipeline_time=2.0)

        # Verify performance validation structure
        assert "performance_validation" in result
        assert result["performance_validation"]["meets_requirements"] is True
        assert result["performance_validation"]["max_allowed_time"] == 2.0
        assert result["performance_validation"]["performance_passes"] is True
        assert result["performance_validation"]["actual_time_seconds"] <= 2.0

    def test_multiple_queries_performance_requirement(self, mock_qdrant_client, mock_cohere_client):
        """Test that multiple queries also meet the performance requirement."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.9,
                "payload": {
                    "content": "Sample content for testing.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "test-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        test_queries = [
            "query about ROS 2",
            "query about middleware concepts",
            "query about node communication"
        ]

        for query in test_queries:
            start_time = time.time()
            result = service.execute_full_pipeline(query, top_k=1, min_score=0.5)
            end_time = time.time()

            execution_time = end_time - start_time

            # Each query should complete within 2 seconds
            assert execution_time <= 2.0, f"Query '{query}' took {execution_time:.2f}s, exceeding 2-second requirement"
            assert result["pipeline_time_seconds"] <= 2.0

    def test_pipeline_performance_at_95_percentile(self, mock_qdrant_client, mock_cohere_client):
        """Test that the pipeline meets performance requirements for 95% of requests."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.88,
                "payload": {
                    "content": "Content for performance testing.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "test-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute multiple queries to test percentile performance
        execution_times = []
        test_queries = [f"test query {i}" for i in range(20)]  # 20 queries for statistical significance

        for query in test_queries:
            start_time = time.time()
            result = service.execute_full_pipeline(query, top_k=1, min_score=0.5)
            end_time = time.time()

            execution_time = end_time - start_time
            execution_times.append(execution_time)

        # Calculate the 95th percentile
        sorted_times = sorted(execution_times)
        p95_index = int(0.95 * len(sorted_times))
        p95_time = sorted_times[min(p95_index, len(sorted_times) - 1)]

        # The 95th percentile should be under 2 seconds
        assert p95_time <= 2.0, f"95th percentile time was {p95_time:.2f}s, exceeding 2-second requirement"

        # Also verify that average time is reasonable
        avg_time = sum(execution_times) / len(execution_times)
        assert avg_time <= 2.0, f"Average time was {avg_time:.2f}s, exceeding 2-second requirement"

    def test_batch_queries_performance_requirement(self, mock_qdrant_client, mock_cohere_client):
        """Test that batch queries meet performance requirements."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.85,
                "payload": {
                    "content": "Content for batch testing.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "test-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test batch execution performance
        queries = ["batch query 1", "batch query 2", "batch query 3"]
        start_time = time.time()
        batch_result = service.execute_batch_queries(queries, top_k=1, min_score=0.5)
        end_time = time.time()

        total_batch_time = end_time - start_time
        avg_query_time = batch_result["summary"]["average_query_time"]

        # Verify that the batch completed in reasonable time
        # For 3 queries, we'd expect it to complete well under 2 seconds per query on average
        assert avg_query_time <= 2.0, f"Average query time in batch was {avg_query_time:.2f}s, exceeding 2-second requirement"
        assert batch_result["summary"]["queries_per_second"] > 0.5  # At least half a query per second

    def test_performance_monitoring_integration(self, mock_qdrant_client, mock_cohere_client):
        """Test that performance is properly monitored and tracked."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.92,
                "payload": {
                    "content": "Content for monitoring test.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "test-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute pipeline multiple times to populate performance monitor
        for i in range(5):
            result = service.execute_full_pipeline(f"monitoring query {i}", top_k=1, min_score=0.5)

        # Get performance stats
        stats = service.performance_monitor.get_stats()

        # Verify performance monitoring is working
        assert stats["total_operations"] >= 5
        assert stats["successful_operations"] >= 5
        assert stats["success_rate"] >= 0.99  # Should be 100% with mocked services
        assert stats["avg_duration"] >= 0
        assert stats["max_duration"] >= stats["min_duration"]

        # Verify that the max duration is within our requirement
        # Note: In a real system, we'd check if p95_duration <= 2.0
        if stats["p95_duration"] > 0:
            assert stats["p95_duration"] <= 2.0, f"95th percentile duration {stats['p95_duration']:.2f}s exceeds requirement"

    def test_performance_requirement_from_specification(self, mock_qdrant_client, mock_cohere_client):
        """Test the specific requirement: 'End-to-end retrieval pipeline completes within 2 seconds for 95% of queries'."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.89,
                "payload": {
                    "content": "ROS 2 uses a sophisticated middleware architecture.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-architecture-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # This test verifies the requirement from the specification:
        # "End-to-end retrieval pipeline completes within 2 seconds for 95% of queries"

        # Run multiple queries to test the 95% requirement
        test_queries = [f"performance requirement test {i}" for i in range(100)]
        execution_times = []

        for query in test_queries:
            start_time = time.time()
            result = service.execute_full_pipeline(query, top_k=1, min_score=0.5)
            end_time = time.time()

            execution_time = end_time - start_time
            execution_times.append(execution_time)

        # Sort times to calculate percentiles
        sorted_times = sorted(execution_times)

        # Calculate 95th percentile (95% of queries should complete within this time)
        p95_index = int(0.95 * len(sorted_times)) - 1
        p95_index = max(0, min(p95_index, len(sorted_times) - 1))
        p95_time = sorted_times[p95_index]

        # Verify the requirement is met: 95% of queries complete within 2 seconds
        assert p95_time <= 2.0, (
            f"Performance requirement failed: 95th percentile time was {p95_time:.2f}s, "
            f"exceeding the 2-second requirement. Only {int(0.95 * len(test_queries))} out of "
            f"{len(test_queries)} queries completed within 2 seconds."
        )

        # Also verify that the percentage is correct
        within_limit_count = sum(1 for t in execution_times if t <= 2.0)
        within_limit_percentage = within_limit_count / len(execution_times)

        assert within_limit_percentage >= 0.95, (
            f"Performance requirement failed: Only {within_limit_percentage:.2%} of queries "
            f"completed within 2 seconds, falling short of the 95% requirement."
        )

        # Additional verification: check that the average time is reasonable
        avg_time = sum(execution_times) / len(execution_times)
        assert avg_time <= 2.0, f"Average execution time {avg_time:.2f}s exceeds 2-second limit"

        print(f"Performance test passed: 95th percentile = {p95_time:.3f}s, "
              f"average = {avg_time:.3f}s, {within_limit_percentage:.1%} within limit")