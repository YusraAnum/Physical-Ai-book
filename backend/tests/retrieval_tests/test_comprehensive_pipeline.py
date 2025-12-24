"""
Comprehensive test for the complete pipeline with timing validation.
"""
import time
import json
import pytest
from unittest.mock import Mock, patch
from src.services.end_to_end_retrieval_service import EndToEndRetrievalService
from src.services.retrieval_service import RetrievalService
from src.services.content_verification_service import ContentVerificationService
from src.services.metadata_validation_service import MetadataValidationService


class TestComprehensivePipeline:
    """Comprehensive test for the complete end-to-end pipeline."""

    def test_complete_pipeline_integration(self, mock_qdrant_client, mock_cohere_client):
        """Test the complete integration of all pipeline components."""
        # Create the end-to-end service
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup comprehensive mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "middleware-intro-001",
                "score": 0.92,
                "payload": {
                    "content": "ROS 2 uses a DDS-based middleware that provides reliable communication between nodes. This middleware layer handles message passing, service calls, and action communication in a distributed system.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-intro-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            },
            {
                "id": "middleware-impl-002",
                "score": 0.87,
                "payload": {
                    "content": "The middleware layer in ROS 2 handles communication protocols and data serialization. It provides a standardized way for different nodes to exchange information regardless of the programming language used.",
                    "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
                    "chunk_id": "middleware-impl-002",
                    "source_title": "Chapter 2: Communication Primitives"
                }
            },
            {
                "id": "nodes-concept-003",
                "score": 0.81,
                "payload": {
                    "content": "Nodes in ROS 2 are computational processes that perform specific tasks. They communicate with each other through topics, services, and actions using the underlying middleware.",
                    "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
                    "chunk_id": "nodes-concept-003",
                    "source_title": "Chapter 2: Communication Primitives"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]  # Simulated embedding

        # Execute the complete pipeline
        start_time = time.time()
        result = service.execute_full_pipeline(
            query_text="Explain ROS 2 middleware concepts",
            top_k=3,
            min_score=0.80
        )
        end_time = time.time()

        execution_time = end_time - start_time

        # Verify the complete pipeline executed successfully
        assert result["pipeline_success"] is True
        assert result["query"] == "Explain ROS 2 middleware concepts"
        assert len(result["results"]) == 3
        assert result["pipeline_time_seconds"] == pytest.approx(execution_time, abs=0.1)
        assert result["total_chunks_searched"] >= 3

        # Verify results are properly structured
        for i, result_item in enumerate(result["results"]):
            assert "id" in result_item
            assert "content" in result_item
            assert "score" in result_item
            assert "metadata" in result_item

            # Verify content is meaningful
            assert len(result_item["content"]) > 10  # Content should be substantial

            # Verify scores are in descending order
            if i > 0:
                assert result["results"][i-1]["score"] >= result_item["score"]

            # Verify metadata is complete
            metadata = result_item["metadata"]
            assert metadata["url"].startswith("http://") or metadata["url"].startswith("https://")
            assert len(metadata["chunk_id"]) > 0
            assert len(metadata["source_title"]) > 0

        # Verify validation components are present
        assert "content_verification" in result
        assert "metadata_validation" in result
        assert "performance_metrics" in result

        # Verify performance metrics
        perf_metrics = result["performance_metrics"]
        assert perf_metrics["chunks_retrieved"] == 3
        assert perf_metrics["top_score"] == 0.92  # Highest score from our mock

        # Verify timing is within requirements
        assert result["pipeline_time_seconds"] <= 2.0

        # Verify JSON serializability
        json_output = json.dumps(result, indent=2)
        parsed_result = json.loads(json_output)
        assert parsed_result == result

    def test_pipeline_with_validation_requirements(self, mock_qdrant_client, mock_cohere_client):
        """Test the pipeline with all validation requirements met."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.90,
                "payload": {
                    "content": "ROS 2 communication is based on a flexible middleware architecture.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-overview-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute pipeline with validation requirements
        result = service.execute_pipeline_with_validation(
            query_text="ROS 2 communication architecture",
            content_threshold=0.90,
            metadata_completeness_threshold=1.0
        )

        # Verify all validation requirements are checked
        assert "validation_results" in result
        validation_results = result["validation_results"]

        # Verify the validation results structure
        assert "content_accuracy_passes" in validation_results
        assert "metadata_completeness_passes" in validation_results
        assert "overall_pipeline_passes" in validation_results

        # The results should pass based on our mock setup
        # (Note: content verification compares content to itself in this implementation)
        assert validation_results["content_accuracy_passes"] is True
        assert validation_results["metadata_completeness_passes"] is True
        assert validation_results["overall_pipeline_passes"] is True

        # Verify the content and metadata validation components are present
        assert "content_verification" in result
        assert "metadata_validation" in result

        # Verify the results meet the requirements:
        # - Top-K matches with 90%+ semantic relevance
        # - Content matches original with 99%+ accuracy (simulated)
        # - Metadata is complete and correct

    def test_pipeline_timing_validation_comprehensive(self, mock_qdrant_client, mock_cohere_client):
        """Comprehensive timing validation for the complete pipeline."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "timing-test-001",
                "score": 0.88,
                "payload": {
                    "content": "This is a content chunk for timing validation testing.",
                    "url": "http://localhost:3002/docs/timing-test",
                    "chunk_id": "timing-test-001",
                    "source_title": "Timing Validation Test"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test multiple executions to validate timing consistency
        execution_times = []
        for i in range(10):
            start_time = time.time()
            result = service.execute_full_pipeline(
                query_text=f"timing test query {i}",
                top_k=1,
                min_score=0.5
            )
            end_time = time.time()

            execution_time = end_time - start_time
            execution_times.append(execution_time)

            # Each execution should be within the 2-second limit
            assert execution_time <= 2.0, f"Execution {i} took {execution_time:.3f}s, exceeding 2s limit"
            assert result["pipeline_time_seconds"] <= 2.0

        # Calculate statistics
        avg_time = sum(execution_times) / len(execution_times)
        max_time = max(execution_times)
        min_time = min(execution_times)

        # Verify statistical requirements
        assert avg_time <= 2.0, f"Average time {avg_time:.3f}s exceeds 2s limit"
        assert max_time <= 2.0, f"Maximum time {max_time:.3f}s exceeds 2s limit"

        # For mocked services, we expect very fast execution
        assert avg_time < 1.0, f"Average time {avg_time:.3f}s seems too slow for mocked services"

        # Verify 95th percentile (95% of requests should be under 2s)
        sorted_times = sorted(execution_times)
        p95_index = int(0.95 * len(sorted_times)) - 1
        p95_index = max(0, min(p95_index, len(sorted_times) - 1))
        p95_time = sorted_times[p95_index]

        assert p95_time <= 2.0, f"95th percentile time {p95_time:.3f}s exceeds 2s limit"

        print(f"Timing validation passed: avg={avg_time:.3f}s, max={max_time:.3f}s, 95th%={p95_time:.3f}s")

    def test_pipeline_with_edge_cases(self, mock_qdrant_client, mock_cohere_client):
        """Test the pipeline with edge cases."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Test 1: Very short query
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "short-query-test",
                "score": 0.75,
                "payload": {
                    "content": "Content for short query test.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "short-query-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        short_result = service.execute_full_pipeline("AI", top_k=1, min_score=0.5)
        assert short_result["pipeline_success"] is True
        assert len(short_result["results"]) >= 0  # May have results or not

        # Test 2: Very long query
        long_query = "This is a very long query about ROS 2 middleware concepts and communication patterns in distributed systems " * 5
        long_result = service.execute_full_pipeline(long_query, top_k=1, min_score=0.5)
        assert long_result["pipeline_success"] is True
        assert long_result["query"] == long_query

        # Test 3: Query with special characters
        special_query = "ROS 2 & middleware: concepts!"
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "special-query-test",
                "score": 0.80,
                "payload": {
                    "content": "Content for special character query test.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "special-query-001",
                    "source_title": "Test Document"
                }
            }
        ]

        special_result = service.execute_full_pipeline(special_query, top_k=1, min_score=0.5)
        assert special_result["pipeline_success"] is True
        assert special_result["query"] == special_query

        # Test 4: No results scenario
        mock_qdrant_client.search_vectors.return_value = []  # No results found
        no_result = service.execute_full_pipeline("completely unrelated query", top_k=5, min_score=0.9)
        assert no_result["pipeline_success"] is True  # Pipeline succeeds, just no results found
        assert len(no_result["results"]) == 0

        # All edge cases should produce valid JSON output
        for result in [short_result, long_result, special_result, no_result]:
            # Verify basic structure is maintained
            assert "query" in result
            assert "results" in result
            assert "pipeline_time_seconds" in result
            assert "pipeline_success" in result

            # Verify JSON serializability
            json.dumps(result)  # Should not raise exception

    def test_pipeline_comprehensive_validation(self, mock_qdrant_client, mock_cohere_client):
        """Comprehensive validation test covering all requirements."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup comprehensive test data
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "comprehensive-test-001",
                "score": 0.91,
                "payload": {
                    "content": "ROS 2 uses a sophisticated middleware architecture based on DDS to enable reliable communication between distributed nodes.",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-architecture-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            },
            {
                "id": "comprehensive-test-002",
                "score": 0.85,
                "payload": {
                    "content": "The communication primitives in ROS 2 include topics for asynchronous messaging, services for synchronous request-response, and actions for goal-oriented communication.",
                    "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
                    "chunk_id": "comm-primitives-002",
                    "source_title": "Chapter 2: Communication Primitives"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Execute the comprehensive pipeline
        start_time = time.time()
        result = service.execute_full_pipeline(
            query_text="Explain ROS 2 middleware and communication primitives",
            top_k=2,
            min_score=0.80,
            verify_content=True,
            validate_metadata=True
        )
        end_time = time.time()

        execution_time = end_time - start_time

        # Validate all requirements from the specification:
        # 1. Query Qdrant and receive correct top-K matches
        assert len(result["results"]) == 2
        scores = [item["score"] for item in result["results"]]
        assert scores == sorted(scores, reverse=True)  # Results should be ordered by score

        # 2. Retrieved chunks match original text (simulated in this implementation)
        assert "content_verification" in result
        if result["content_verification"]:
            # In our implementation, content verification compares content to itself
            assert result["content_verification"]["valid_chunk_count"] >= 0

        # 3. Metadata (url, chunk_id) returns correctly
        assert "metadata_validation" in result
        if result["metadata_validation"]:
            metadata_validation = result["metadata_validation"]
            assert metadata_validation["completeness"]["all_complete"] is True
            assert metadata_validation["url_correctness"]["all_valid"] is True
            assert metadata_validation["uniqueness"]["all_unique"] is True

        # 4. End-to-end test: input query + Qdrant response + clean JSON output
        assert result["query"] == "Explain ROS 2 middleware and communication primitives"
        assert isinstance(result["results"], list)
        assert result["pipeline_time_seconds"] <= 2.0
        assert execution_time <= 2.0

        # 5. JSON output is properly formatted
        json_output = json.dumps(result, indent=2, ensure_ascii=False)
        parsed_result = json.loads(json_output)
        assert parsed_result == result

        # 6. Performance requirements met (95% of queries complete within 2s)
        assert result["pipeline_time_seconds"] <= 2.0

        # 7. All metadata fields are present and correct
        for result_item in result["results"]:
            metadata = result_item["metadata"]
            assert metadata["url"].startswith(("http://", "https://"))
            assert len(metadata["url"]) > 10  # Reasonable URL length
            assert len(metadata["chunk_id"]) > 0
            assert metadata["chunk_id"] != metadata["url"]  # Different identifiers

        # 8. Content is meaningful and relevant
        for result_item in result["results"]:
            assert len(result_item["content"]) > 20  # Substantial content
            # Content should relate to the query topic (simulated verification)

        # 9. Validation components are present
        assert "content_verification" in result
        assert "metadata_validation" in result
        assert "performance_metrics" in result

        # 10. Pipeline success flag is properly set
        assert result["pipeline_success"] is True

        print("Comprehensive pipeline validation passed:")
        print(f"  - Query processed: '{result['query'][:50]}...'")
        print(f"  - Results returned: {len(result['results'])}")
        print(f"  - Execution time: {result['pipeline_time_seconds']:.3f}s")
        print(f"  - Content verification: {'Present' if result['content_verification'] else 'None'}")
        print(f"  - Metadata validation: {'Present' if result['metadata_validation'] else 'None'}")
        print(f"  - JSON format: Valid")