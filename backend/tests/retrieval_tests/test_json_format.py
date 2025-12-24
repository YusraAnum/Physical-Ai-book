"""
Tests to validate the JSON output format as specified in requirements.
"""
import json
import pytest
from unittest.mock import Mock, patch
from src.services.end_to_end_retrieval_service import EndToEndRetrievalService


class TestJSONFormatValidation:
    """Tests to validate the JSON output format."""

    def test_clean_json_output_with_top_k_matches(self, mock_qdrant_client, mock_cohere_client):
        """Test that the output contains clean JSON with top-K matches."""
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
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute the pipeline
        result = service.execute_full_pipeline("ROS 2 middleware concepts", top_k=2, min_score=0.5)

        # Verify the result is a valid dictionary (which can be serialized to JSON)
        assert isinstance(result, dict)

        # Verify required top-level fields exist
        assert "query" in result
        assert "results" in result
        assert "pipeline_time_seconds" in result
        assert "pipeline_time_ms" in result
        assert "total_chunks_searched" in result

        # Verify results structure
        assert isinstance(result["results"], list)
        assert len(result["results"]) == 2  # Should have 2 results as requested

        # Verify each result has required fields
        for result_item in result["results"]:
            assert "id" in result_item
            assert "content" in result_item
            assert "score" in result_item
            assert "metadata" in result_item

            # Verify metadata structure
            metadata = result_item["metadata"]
            assert "url" in metadata
            assert "chunk_id" in metadata
            assert "source_title" in metadata

        # Verify scores are in descending order (top-K matches)
        scores = [item["score"] for item in result["results"]]
        assert scores == sorted(scores, reverse=True), "Results should be ordered by score (descending)"

        # Verify the JSON can be serialized without errors
        try:
            json_str = json.dumps(result)
            parsed_back = json.loads(json_str)
            assert parsed_back == result  # Ensure no data loss in serialization
        except (TypeError, ValueError) as e:
            pytest.fail(f"Result cannot be serialized to JSON: {e}")

    def test_json_output_consistency_for_100_percent_success(self, mock_qdrant_client, mock_cohere_client):
        """Test that JSON output is consistent for 100% of successful retrieval requests."""
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
                    "content": "Sample content for consistency test.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "test-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test multiple queries to ensure consistency
        test_queries = ["query 1", "query 2", "query 3", "query 4", "query 5"]

        for query in test_queries:
            result = service.execute_full_pipeline(query, top_k=1, min_score=0.5)

            # Verify consistent structure across all queries
            assert isinstance(result, dict)
            assert "query" in result
            assert "results" in result
            assert "pipeline_time_seconds" in result
            assert "pipeline_success" in result

            # Verify the query field matches the input
            assert result["query"] == query

            # Verify results structure
            assert isinstance(result["results"], list)

            # Verify JSON serializability for each result
            try:
                json_str = json.dumps(result, ensure_ascii=False)
                parsed_back = json.loads(json_str)
                assert parsed_back == result
            except (TypeError, ValueError) as e:
                pytest.fail(f"Query '{query}' result cannot be serialized to JSON: {e}")

    def test_json_format_with_empty_results(self, mock_qdrant_client, mock_cohere_client):
        """Test JSON format when no results are returned."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock to return no results
        mock_qdrant_client.search_vectors.return_value = []
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute the pipeline
        result = service.execute_full_pipeline("unrelated query", top_k=5, min_score=0.8)

        # Verify structure is maintained even with no results
        assert isinstance(result, dict)
        assert "query" in result
        assert "results" in result
        assert "pipeline_time_seconds" in result
        assert "pipeline_success" in result

        # Results should be an empty list, not null or missing
        assert isinstance(result["results"], list)
        assert len(result["results"]) == 0

        # Pipeline should still be marked as successful (it's not an error to have no results)
        assert result["pipeline_success"] is True

        # Verify JSON serializability
        try:
            json_str = json.dumps(result)
            parsed_back = json.loads(json_str)
            assert parsed_back == result
        except (TypeError, ValueError) as e:
            pytest.fail(f"Empty results cannot be serialized to JSON: {e}")

    def test_json_format_with_validation_components(self, mock_qdrant_client, mock_cohere_client):
        """Test JSON format includes validation components."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.91,
                "payload": {
                    "content": "Content with validation.",
                    "url": "http://localhost:3002/docs/test",
                    "chunk_id": "validation-chunk-001",
                    "source_title": "Test Document"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute pipeline with validation
        result = service.execute_pipeline_with_validation(
            "validation test query",
            content_threshold=0.90,
            metadata_completeness_threshold=0.90
        )

        # Verify validation components are included in JSON
        assert "validation_results" in result
        assert "content_verification" in result
        assert "metadata_validation" in result

        # Verify validation structure
        validation_results = result["validation_results"]
        assert "content_accuracy_passes" in validation_results
        assert "metadata_completeness_passes" in validation_results
        assert "overall_pipeline_passes" in validation_results

        # Verify JSON serializability
        try:
            json_str = json.dumps(result)
            parsed_back = json.loads(json_str)
            assert parsed_back == result
        except (TypeError, ValueError) as e:
            pytest.fail(f"Validation results cannot be serialized to JSON: {e}")

    def test_json_format_requirements_validation(self, mock_qdrant_client, mock_cohere_client):
        """Test the specific requirement: 'JSON output is properly formatted and consistent for 100% of successful retrieval requests'."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # This test verifies the requirement from the specification:
        # "JSON output is properly formatted and consistent for 100% of successful retrieval requests"

        # Setup mock results
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.89,
                "payload": {
                    "content": "ROS 2 communication primitives include topics, services, and actions.",
                    "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
                    "chunk_id": "comm-primitives-001",
                    "source_title": "Chapter 2: Communication Primitives"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Test multiple retrieval requests to verify 100% success rate
        test_queries = [
            "ROS 2 communication primitives",
            "middleware architecture",
            "node communication patterns",
            "topic and service usage",
            "action communication in ROS 2"
        ]

        successful_requests = 0
        total_requests = len(test_queries)

        for query in test_queries:
            try:
                result = service.execute_full_pipeline(query, top_k=1, min_score=0.5)

                # Verify basic JSON structure
                assert isinstance(result, dict), f"Result for query '{query}' is not a dictionary"
                assert "query" in result, f"Result for query '{query}' missing 'query' field"
                assert "results" in result, f"Result for query '{query}' missing 'results' field"
                assert "pipeline_time_seconds" in result, f"Result for query '{query}' missing 'pipeline_time_seconds' field"
                assert isinstance(result["results"], list), f"Results for query '{query}' is not a list"

                # Verify JSON serializability
                json_str = json.dumps(result, ensure_ascii=False)
                parsed_back = json.loads(json_str)
                assert parsed_back == result, f"JSON serialization for query '{query}' is inconsistent"

                # Verify consistent field presence
                required_fields = ["query", "results", "pipeline_time_seconds", "pipeline_time_ms",
                                 "total_chunks_searched", "content_verification", "metadata_validation",
                                 "pipeline_success", "performance_metrics"]

                for field in required_fields:
                    assert field in result, f"Result for query '{query}' missing required field: {field}"

                # Verify results structure if results exist
                for result_item in result["results"]:
                    required_result_fields = ["id", "content", "score", "metadata"]
                    for field in required_result_fields:
                        assert field in result_item, f"Result item for query '{query}' missing field: {field}"

                    # Verify metadata structure
                    metadata = result_item["metadata"]
                    metadata_required_fields = ["url", "chunk_id", "source_title"]
                    for field in metadata_required_fields:
                        assert field in metadata, f"Metadata for query '{query}' missing field: {field}"

                successful_requests += 1

            except Exception as e:
                pytest.fail(f"Query '{query}' failed JSON format validation: {e}")

        # Verify 100% success rate
        success_rate = successful_requests / total_requests
        assert success_rate == 1.0, f"Only {successful_requests}/{total_requests} ({success_rate:.0%}) requests had proper JSON format"

        print(f"JSON format validation passed: {successful_requests}/{total_requests} requests ({success_rate:.0%} success rate)")

    def test_json_format_with_all_specified_fields(self, mock_qdrant_client, mock_cohere_client):
        """Test that JSON output contains all fields specified in the API contract."""
        service = EndToEndRetrievalService()

        # Replace the internal services with mocked ones
        service.retrieval_service.qdrant_client = mock_qdrant_client
        service.retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results matching the API contract example
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-123",
                "score": 0.92,
                "payload": {
                    "content": "ROS 2 uses a DDS-based middleware that provides...",
                    "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
                    "chunk_id": "middleware-intro-001",
                    "source_title": "Chapter 1: ROS 2 as Middleware"
                }
            },
            {
                "id": "chunk-456",
                "score": 0.87,
                "payload": {
                    "content": "The middleware layer in ROS 2 handles communication...",
                    "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
                    "chunk_id": "middleware-impl-002",
                    "source_title": "Chapter 2: Communication Primitives"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Execute the pipeline
        result = service.execute_full_pipeline(
            "Explain ROS 2 middleware concepts",
            top_k=2,
            min_score=0.6
        )

        # Verify all fields from the API contract are present
        assert result["query"] == "Explain ROS 2 middleware concepts"

        # Verify results structure matches API contract
        assert len(result["results"]) == 2
        first_result = result["results"][0]

        # Verify first result matches API contract structure
        assert "id" in first_result
        assert "content" in first_result
        assert "score" in first_result
        assert "metadata" in first_result

        # Verify metadata structure
        metadata = first_result["metadata"]
        assert "url" in metadata
        assert "chunk_id" in metadata
        assert "source_title" in metadata

        # Verify timing information
        assert "pipeline_time_ms" in result
        assert "total_chunks_searched" in result

        # Verify the output can be validated against the Pydantic models
        # (Implicitly tested by the fact that it's properly structured)

        # Verify JSON serializability
        json_output = json.dumps(result, indent=2)
        parsed_result = json.loads(json_output)

        # Verify no data was lost in serialization
        assert parsed_result["query"] == result["query"]
        assert len(parsed_result["results"]) == len(result["results"])
        assert parsed_result["pipeline_time_ms"] == result["pipeline_time_ms"]

        print("JSON format validation passed: All API contract fields present and properly structured")