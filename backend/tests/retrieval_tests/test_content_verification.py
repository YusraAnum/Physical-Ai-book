"""
Unit tests for the ContentVerificationService class.
"""
import pytest
from src.services.content_verification_service import ContentVerificationService


class TestContentVerificationService:
    """Unit tests for ContentVerificationService."""

    def test_verify_content_accuracy_exact_match(self):
        """Test content verification with exact match."""
        service = ContentVerificationService()

        retrieved = "This is the exact content."
        original = "This is the exact content."

        is_accurate, similarity, details = service.verify_content_accuracy(retrieved, original, threshold=0.99)

        assert is_accurate is True
        assert similarity == 1.0
        assert details["content_matches"] is True

    def test_verify_content_accuracy_with_threshold(self):
        """Test content verification with similarity above threshold."""
        service = ContentVerificationService()

        retrieved = "This is the content with minor changes."
        original = "This is the content with minor change."

        is_accurate, similarity, details = service.verify_content_accuracy(retrieved, original, threshold=0.90)

        assert is_accurate is True
        assert similarity > 0.90
        assert details["content_matches"] is True

    def test_verify_content_accuracy_below_threshold(self):
        """Test content verification with similarity below threshold."""
        service = ContentVerificationService()

        retrieved = "This is completely different content."
        original = "This is the original content."

        is_accurate, similarity, details = service.verify_content_accuracy(retrieved, original, threshold=0.80)

        assert is_accurate is False
        assert similarity < 0.80
        assert details["content_matches"] is False

    def test_verify_content_accuracy_empty_strings(self):
        """Test content verification with empty strings."""
        service = ContentVerificationService()

        # Both empty
        is_accurate, similarity, details = service.verify_content_accuracy("", "", threshold=0.99)
        assert is_accurate is True
        assert similarity == 1.0
        assert details["match_type"] == "both_empty"

        # One empty
        is_accurate, similarity, details = service.verify_content_accuracy("content", "", threshold=0.99)
        assert is_accurate is False
        assert similarity == 0.0
        assert details["match_type"] == "one_empty"

    def test_calculate_content_similarity_identical(self):
        """Test content similarity calculation for identical content."""
        service = ContentVerificationService()

        content1 = "This is identical content."
        content2 = "This is identical content."

        similarity = service.calculate_content_similarity(content1, content2)

        assert similarity == 1.0

    def test_calculate_content_similarity_different(self):
        """Test content similarity calculation for different content."""
        service = ContentVerificationService()

        content1 = "This is the first content."
        content2 = "This is the second content."

        similarity = service.calculate_content_similarity(content1, content2)

        assert 0.0 < similarity < 1.0

    def test_calculate_content_similarity_empty(self):
        """Test content similarity calculation with empty content."""
        service = ContentVerificationService()

        # Both empty
        similarity = service.calculate_content_similarity("", "")
        assert similarity == 1.0

        # One empty
        similarity = service.calculate_content_similarity("content", "")
        assert similarity == 0.0

    def test_verify_content_integrity_matching_hashes(self):
        """Test content integrity verification with matching hashes."""
        service = ContentVerificationService()

        content = "This is the content to verify."
        is_valid, actual_hash = service.verify_content_integrity(content, expected_hash=None)

        # When expected_hash is None, it should return True
        assert is_valid is True
        assert actual_hash is not None

        # Verify with the actual hash
        is_valid, actual_hash2 = service.verify_content_integrity(content, expected_hash=actual_hash)
        assert is_valid is True
        assert actual_hash == actual_hash2

    def test_verify_content_integrity_different_content(self):
        """Test content integrity verification with different content."""
        service = ContentVerificationService()

        content1 = "This is the first content."
        content2 = "This is the second content."

        # Get hash of first content
        _, hash1 = service.verify_content_integrity(content1)

        # Verify second content against first content's hash
        is_valid, hash2 = service.verify_content_integrity(content2, hash1)

        assert is_valid is False
        assert hash1 != hash2

    def test_compare_content_structures_matching(self):
        """Test content structure comparison for matching content."""
        service = ContentVerificationService()

        content1 = "Line 1\nLine 2\nLine 3"
        content2 = "Line 1\nLine 2\nLine 3"

        structure = service.compare_content_structures(content1, content2)

        assert structure["line_count_match"] is True
        assert structure["word_count_match"] is True
        assert structure["line_similarity"] == 1.0

    def test_compare_content_structures_different(self):
        """Test content structure comparison for different content."""
        service = ContentVerificationService()

        original_content = "Line 1\nLine 2\nLine 3"
        retrieved_content = "Line A\nLine B"

        structure = service.compare_content_structures(retrieved_content, original_content)

        assert structure["line_count_match"] is False
        assert structure["original_line_count"] == 3
        assert structure["retrieved_line_count"] == 2

    def test_get_content_differences_present(self):
        """Test getting content differences when differences exist."""
        service = ContentVerificationService()

        original = "Line 1\nLine 2\nLine 3"
        retrieved = "Line 1\nLine X\nLine 3"

        diff_info = service.get_content_differences(retrieved, original)

        assert diff_info["has_differences"] is True
        assert diff_info["total_differences"] > 0
        assert 0 <= diff_info["similarity_percentage"] <= 100

    def test_get_content_differences_none(self):
        """Test getting content differences when no differences exist."""
        service = ContentVerificationService()

        content = "Line 1\nLine 2\nLine 3"

        diff_info = service.get_content_differences(content, content)

        assert diff_info["has_differences"] is False
        assert diff_info["total_differences"] == 0
        assert diff_info["similarity_percentage"] == 100.0

    def test_validate_retrieval_accuracy_all_match(self):
        """Test retrieval accuracy validation when all chunks match."""
        service = ContentVerificationService()

        retrieved_chunks = ["Content A", "Content B"]
        original_chunks = ["Content A", "Content B"]

        result = service.validate_retrieval_accuracy(retrieved_chunks, original_chunks, threshold=0.99)

        assert result["accuracy_valid"] is True
        assert result["overall_similarity"] == 1.0
        assert result["chunk_count_match"] is True
        assert result["accuracy_percentage"] == 100.0

    def test_validate_retrieval_accuracy_partial_match(self):
        """Test retrieval accuracy validation when some chunks don't match."""
        service = ContentVerificationService()

        retrieved_chunks = ["Content A", "Different Content"]
        original_chunks = ["Content A", "Original Content"]

        result = service.validate_retrieval_accuracy(retrieved_chunks, original_chunks, threshold=0.99)

        assert result["accuracy_valid"] is False  # Because one chunk doesn't match
        assert 0.0 <= result["overall_similarity"] < 1.0
        assert result["chunk_count_match"] is True
        assert result["accuracy_percentage"] == 50.0  # Only 1 out of 2 chunks match

    def test_validate_retrieval_accuracy_count_mismatch(self):
        """Test retrieval accuracy validation when chunk counts don't match."""
        service = ContentVerificationService()

        retrieved_chunks = ["Content A"]
        original_chunks = ["Content A", "Content B"]

        result = service.validate_retrieval_accuracy(retrieved_chunks, original_chunks, threshold=0.99)

        assert result["accuracy_valid"] is False
        assert result["chunk_count_match"] is False
        assert result["total_chunk_count"] == 1  # Based on retrieved chunks