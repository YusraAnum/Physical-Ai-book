"""
Unit tests for the MetadataValidationService class.
"""
import pytest
from src.models.metadata import Metadata
from src.services.metadata_validation_service import MetadataValidationService


class TestMetadataValidationService:
    """Unit tests for MetadataValidationService."""

    def test_validate_metadata_completeness_all_complete(self):
        """Test metadata completeness validation when all metadata is complete."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1", source_title="Document 1"),
            Metadata(url="http://example.com/doc2", chunk_id="chunk-2", source_title="Document 2")
        ]

        result = service.validate_metadata_completeness(metadata_list)

        assert result["all_complete"] is True
        assert result["total_count"] == 2
        assert result["complete_count"] == 2
        assert result["completeness_percentage"] == 100.0
        assert all(detail["is_complete"] for detail in result["validation_details"])

    def test_validate_metadata_completeness_with_missing_fields(self):
        """Test metadata completeness validation with missing required fields."""
        service = MetadataValidationService()

        # Create metadata with missing required fields
        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1", source_title="Document 1"),
            Metadata(url="", chunk_id="chunk-2", source_title="Document 2"),  # Missing URL
            Metadata(url="http://example.com/doc3", chunk_id="", source_title="Document 3")  # Missing chunk_id
        ]

        result = service.validate_metadata_completeness(metadata_list)

        assert result["all_complete"] is False
        assert result["total_count"] == 3
        assert result["complete_count"] == 1  # Only first one is complete
        assert abs(result["completeness_percentage"] - 33.33) < 0.02  # Approximately 33.33
        assert result["validation_details"][0]["is_complete"] is True
        assert result["validation_details"][1]["is_complete"] is False
        assert result["validation_details"][2]["is_complete"] is False

    def test_validate_url_correctness_all_valid(self):
        """Test URL correctness validation when all URLs are valid."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1"),
            Metadata(url="https://example.com/doc2", chunk_id="chunk-2"),
            Metadata(url="http://localhost:3000/doc3", chunk_id="chunk-3")
        ]

        result = service.validate_url_correctness(metadata_list)

        assert result["all_valid"] is True
        assert result["total_count"] == 3
        assert result["valid_count"] == 3
        assert result["valid_percentage"] == 100.0
        assert all(detail["is_valid"] for detail in result["validation_details"])

    def test_validate_url_correctness_with_invalid_urls(self):
        """Test URL correctness validation with invalid URLs."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1"),
            Metadata(url="invalid-url", chunk_id="chunk-2"),
            Metadata(url="", chunk_id="chunk-3"),  # Empty URL
            Metadata(url="ftp://example.com/doc4", chunk_id="chunk-4")  # Valid format but might be considered invalid in some contexts
        ]

        result = service.validate_url_correctness(metadata_list)

        assert result["all_valid"] is False
        assert result["total_count"] == 4
        assert result["valid_count"] == 1  # Only the first (http) URL is valid
        assert result["valid_percentage"] == 25.0

    def test_validate_chunk_id_uniqueness_all_unique(self):
        """Test chunk_id uniqueness validation when all chunk_ids are unique."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1"),
            Metadata(url="http://example.com/doc2", chunk_id="chunk-2"),
            Metadata(url="http://example.com/doc3", chunk_id="chunk-3")
        ]

        result = service.validate_chunk_id_uniqueness(metadata_list)

        assert result["all_unique"] is True
        assert result["total_count"] == 3
        assert result["duplicate_count"] == 0
        assert result["unique_count"] == 3
        assert len(result["duplicates"]) == 0

    def test_validate_chunk_id_uniqueness_with_duplicates(self):
        """Test chunk_id uniqueness validation with duplicate chunk_ids."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1"),
            Metadata(url="http://example.com/doc2", chunk_id="chunk-2"),
            Metadata(url="http://example.com/doc3", chunk_id="chunk-1"),  # Duplicate
            Metadata(url="http://example.com/doc4", chunk_id="chunk-2"),  # Duplicate
            Metadata(url="http://example.com/doc5", chunk_id="chunk-3")
        ]

        result = service.validate_chunk_id_uniqueness(metadata_list)

        assert result["all_unique"] is False
        assert result["total_count"] == 5
        assert result["duplicate_count"] == 2  # Two duplicates
        assert result["unique_count"] == 3  # Three unique IDs
        assert "chunk-1" in result["duplicates"]
        assert "chunk-2" in result["duplicates"]

    def test_validate_metadata_consistency_no_issues(self):
        """Test metadata consistency validation with no issues."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1", source_title="Document 1"),
            Metadata(url="https://example.com/doc2", chunk_id="chunk-2", source_title="Document 2")
        ]

        result = service.validate_metadata_consistency(metadata_list)

        assert result["consistent"] is True
        assert len(result["issues"]) == 0

    def test_validate_metadata_consistency_with_issues(self):
        """Test metadata consistency validation with issues."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1", source_title="Document 1"),
            Metadata(url="", chunk_id="chunk-2", source_title="Document 2"),  # Empty URL
            Metadata(url="example.com/doc3", chunk_id="chunk-3", source_title="Document 3"),  # Missing protocol
            Metadata(url="http://example.com/doc4", chunk_id="", source_title="Document 4"),  # Empty chunk_id
            Metadata(url="  ", chunk_id="chunk-5", source_title="Document 5"),  # Whitespace-only URL
        ]

        result = service.validate_metadata_consistency(metadata_list)

        assert result["consistent"] is False
        assert len(result["issues"]) == 5  # Five issues expected (last entry has 2 issues: empty URL and missing protocol)

        # Check specific issues
        issue_fields = [issue["field"] for issue in result["issues"]]
        assert issue_fields.count("url") == 4  # Empty URL (2 instances), missing protocol (2 instances)
        assert issue_fields.count("chunk_id") == 1  # Empty chunk_id

    def test_validate_metadata_accessibility_all_accessible(self):
        """Test metadata accessibility validation when all metadata is accessible."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1"),
            Metadata(url="https://example.com/doc2", chunk_id="chunk-2")
        ]

        result = service.validate_metadata_accessibility(metadata_list)

        assert result["accessible"] is True
        assert result["total_count"] == 2
        assert result["accessible_count"] == 2
        assert result["accessibility_percentage"] == 100.0
        assert len(result["accessibility_issues"]) == 0

    def test_validate_metadata_accessibility_with_inaccessible(self):
        """Test metadata accessibility validation with inaccessible metadata."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1"),
            Metadata(url="", chunk_id="chunk-2"),  # Inaccessible - empty URL
            Metadata(url="http://example.com/doc3", chunk_id=""),  # Inaccessible - empty chunk_id
            Metadata(url="  ", chunk_id="chunk-4"),  # Inaccessible - whitespace URL
        ]

        result = service.validate_metadata_accessibility(metadata_list)

        assert result["accessible"] is False
        assert result["total_count"] == 4
        assert result["accessible_count"] == 1  # Only first one is accessible
        assert result["accessibility_percentage"] == 25.0
        assert len(result["accessibility_issues"]) == 3

    def test_comprehensive_metadata_validation_all_valid(self):
        """Test comprehensive metadata validation when all validations pass."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1", source_title="Document 1"),
            Metadata(url="https://example.com/doc2", chunk_id="chunk-2", source_title="Document 2")
        ]

        result = service.comprehensive_metadata_validation(metadata_list)

        assert result["overall_valid"] is True
        assert result["completeness"]["all_complete"] is True
        assert result["url_correctness"]["all_valid"] is True
        assert result["uniqueness"]["all_unique"] is True
        assert result["consistency"]["consistent"] is True
        assert result["accessibility"]["accessible"] is True

    def test_comprehensive_metadata_validation_with_issues(self):
        """Test comprehensive metadata validation with various issues."""
        service = MetadataValidationService()

        metadata_list = [
            Metadata(url="http://example.com/doc1", chunk_id="chunk-1", source_title="Document 1"),
            Metadata(url="", chunk_id="chunk-2", source_title="Document 2"),  # Missing URL
            Metadata(url="http://example.com/doc3", chunk_id="chunk-1", source_title="Document 3"),  # Duplicate chunk_id
            Metadata(url="invalid-url", chunk_id="chunk-4", source_title="Document 4"),  # Invalid URL
        ]

        result = service.comprehensive_metadata_validation(metadata_list)

        assert result["overall_valid"] is False
        assert result["summary"]["total_metadata_count"] == 4
        assert result["summary"]["validation_passed"] is False
        assert result["summary"]["completeness_percentage"] < 100.0
        assert result["summary"]["url_validity_percentage"] < 100.0
        assert result["summary"]["accessibility_percentage"] < 100.0

    def test_empty_metadata_list_validation(self):
        """Test validation with empty metadata list."""
        service = MetadataValidationService()

        result = service.validate_metadata_completeness([])
        assert result["all_complete"] is True  # Vacuous truth: all 0 items are complete
        assert result["total_count"] == 0
        assert result["complete_count"] == 0
        assert result["completeness_percentage"] == 100.0

        result = service.validate_url_correctness([])
        assert result["all_valid"] is True  # Empty list is considered valid
        assert result["total_count"] == 0

        result = service.validate_chunk_id_uniqueness([])
        assert result["all_unique"] is True  # Empty list is considered unique
        assert result["total_count"] == 0

        result = service.validate_metadata_consistency([])
        assert result["consistent"] is True  # Empty list is considered consistent
        assert len(result["issues"]) == 0

        result = service.validate_metadata_accessibility([])
        assert result["accessible"] is True  # Empty list is considered accessible
        assert result["total_count"] == 0

        result = service.comprehensive_metadata_validation([])
        assert result["overall_valid"] is True  # Empty list passes all validations
        assert result["summary"]["total_metadata_count"] == 0