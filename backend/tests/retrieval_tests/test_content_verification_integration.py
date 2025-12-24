"""
Integration tests for content verification functionality.
"""
import pytest
from unittest.mock import Mock, patch
from src.services.retrieval_service import RetrievalService
from src.services.content_verification_service import ContentVerificationService
from src.utils.content_verification_utils import (
    normalize_text,
    compare_text_segments,
    calculate_text_fingerprint,
    compare_text_fingerprints,
    validate_content_within_tolerance
)


class TestContentVerificationIntegration:
    """Integration tests for content verification functionality."""

    def test_retrieval_with_content_verification(self, mock_qdrant_client, mock_cohere_client):
        """Test retrieval service with integrated content verification."""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        service.cohere_client = mock_cohere_client

        # Setup mock to return a result
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.9,
                "payload": {
                    "content": "ROS 2 uses a DDS-based middleware that provides reliable communication.",
                    "url": "test.com",
                    "chunk_id": "1"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Perform retrieval
        result = service.get_top_k_matches("ROS 2 middleware", top_k=1, min_score=0.5)

        # Verify that we got results
        assert len(result.matches) == 1
        assert result.matches[0].content is not None

        # Test content verification on the result
        verification_service = ContentVerificationService()
        is_accurate, similarity, details = verification_service.verify_content_accuracy(
            result.matches[0].content,
            "ROS 2 uses a DDS-based middleware that provides reliable communication.",
            threshold=0.95
        )

        assert is_accurate is True
        assert similarity > 0.95

    def test_content_verification_with_normalization(self):
        """Test content verification with text normalization."""
        verification_service = ContentVerificationService()

        # Content with different formatting but same meaning
        retrieved = "  This   is   the   content.  \nWith   extra   spaces.  "
        original = "This is the content.\nWith extra spaces."

        # Test without normalization (should have lower similarity)
        is_accurate, similarity, details = verification_service.verify_content_accuracy(
            retrieved, original, threshold=0.95
        )

        # Test with normalization
        norm_retrieved = normalize_text(retrieved)
        norm_original = normalize_text(original)

        is_accurate_norm, similarity_norm, details_norm = verification_service.verify_content_accuracy(
            norm_retrieved, norm_original, threshold=0.95
        )

        # Normalized version should have higher similarity
        assert similarity_norm >= similarity
        assert is_accurate_norm is True

    def test_segment_based_comparison(self):
        """Test segment-based content comparison."""
        long_content1 = "This is the first part of the content. " * 10  # Repeat to make it long
        long_content2 = "This is the first part of the content. " * 9 + "This is the last part."

        # Compare using segments
        from src.utils.content_verification_utils import extract_text_segments
        segments1 = extract_text_segments(long_content1, segment_length=50)
        segments2 = extract_text_segments(long_content2, segment_length=50)

        comparison = compare_text_segments(segments1, segments2, threshold=0.8)

        assert "matching_segments" in comparison
        assert "total_segments" in comparison
        assert comparison["total_segments"] > 0

    def test_text_fingerprint_comparison(self):
        """Test text fingerprint-based comparison."""
        content1 = "This is a sample content with several words and sentences. It has a specific structure."
        content2 = "This is a sample content with several words and sentences. It has a specific structure."

        fp1 = calculate_text_fingerprint(content1)
        fp2 = calculate_text_fingerprint(content2)

        # Compare fingerprints
        comparison = compare_text_fingerprints(fp1, fp2, tolerance=0.05)

        # Most characteristics should match
        matching_characteristics = sum(1 for match in comparison.values() if match)
        total_characteristics = len(comparison)

        # At least most characteristics should match for similar content
        assert matching_characteristics / total_characteristics >= 0.8

    def test_tolerance_based_validation(self):
        """Test content validation with different tolerance levels."""
        content1 = "This is the original content with some specific wording."
        content2 = "This is the original content with slightly different wording."

        # Test with different tolerance levels
        strict_valid, strict_details = validate_content_within_tolerance(content1, content2, "strict")
        moderate_valid, moderate_details = validate_content_within_tolerance(content1, content2, "moderate")
        lenient_valid, lenient_details = validate_content_within_tolerance(content1, content2, "lenient")

        # Lenient should be more permissive than strict
        assert lenient_details["tolerance_level"] == "lenient"
        assert moderate_details["tolerance_level"] == "moderate"
        assert strict_details["tolerance_level"] == "strict"

    def test_end_to_end_content_verification_flow(self, mock_qdrant_client, mock_cohere_client):
        """Test end-to-end content verification flow."""
        # Setup retrieval service
        retrieval_service = RetrievalService()
        retrieval_service.qdrant_client = mock_qdrant_client
        retrieval_service.cohere_client = mock_cohere_client

        # Setup mock results
        original_content = "ROS 2 uses a sophisticated middleware architecture based on DDS."
        mock_qdrant_client.search_vectors.return_value = [
            {
                "id": "chunk-1",
                "score": 0.85,
                "payload": {
                    "content": original_content,  # In this case, retrieved matches original
                    "url": "http://test.com/doc",
                    "chunk_id": "chunk-001"
                }
            }
        ]
        mock_cohere_client.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Step 1: Retrieve content
        retrieval_result = retrieval_service.get_top_k_matches("ROS 2 middleware", top_k=1, min_score=0.5)

        assert len(retrieval_result.matches) == 1
        retrieved_content = retrieval_result.matches[0].content

        # Step 2: Verify content accuracy
        verification_service = ContentVerificationService()
        is_accurate, similarity, details = verification_service.verify_content_accuracy(
            retrieved_content, original_content, threshold=0.95
        )

        assert is_accurate is True
        assert similarity > 0.95

        # Step 3: Perform additional verification checks
        structure_comparison = verification_service.compare_content_structures(
            retrieved_content, original_content
        )

        assert structure_comparison["word_count_match"] is True

        # Step 4: Get detailed differences (should be minimal)
        diff_info = verification_service.get_content_differences(
            retrieved_content, original_content
        )

        assert diff_info["similarity_percentage"] > 95
        assert diff_info["has_differences"] is False  # Since content is identical in this test

    def test_content_verification_with_varied_content(self):
        """Test content verification with different types of content variations."""
        verification_service = ContentVerificationService()

        # Test cases with different types of variations
        test_cases = [
            # Slight variation
            {
                "retrieved": "The system uses a middleware for communication.",
                "original": "The system uses middleware for communication.",
                "threshold": 0.90,
                "should_match": True
            },
            # Significant difference
            {
                "retrieved": "This is about database systems.",
                "original": "This is about network protocols.",
                "threshold": 0.50,
                "should_match": False
            },
            # Same content, different formatting
            {
                "retrieved": "ROS 2\nuses DDS.",
                "original": "ROS 2 uses DDS.",
                "threshold": 0.95,
                "should_match": True  # Should match after normalization
            }
        ]

        for i, case in enumerate(test_cases):
            is_accurate, similarity, details = verification_service.verify_content_accuracy(
                case["retrieved"], case["original"], case["threshold"]
            )

            if case["should_match"]:
                assert is_accurate is True, f"Test case {i} should match but didn't"
            else:
                assert is_accurate is False, f"Test case {i} should not match but did"