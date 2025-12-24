"""
Tests to verify URL and chunk_id correctness in metadata as specified in requirements.
"""
import pytest
from src.models.metadata import Metadata
from src.services.metadata_validation_service import MetadataValidationService


class TestMetadataCorrectness:
    """Tests for verifying URL and chunk_id correctness."""

    def test_url_correctly_identifies_source_document(self):
        """Test that URL correctly identifies the source document."""
        service = MetadataValidationService()

        # Create metadata with proper URLs that identify source documents
        metadata_list = [
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-intro-001",
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-2/nodes-topics-services",
                chunk_id="services-concept-005",
                source_title="Chapter 2: Communication Primitives"
            )
        ]

        # Validate that URLs are correct
        url_validation = service.validate_url_correctness(metadata_list)

        assert url_validation["all_valid"] is True
        assert url_validation["valid_percentage"] == 100.0

        # Check that each URL properly identifies a source document
        for i, metadata in enumerate(metadata_list):
            assert metadata.url is not None
            assert metadata.url.strip() != ""
            assert metadata.source_title is not None
            # Verify URL follows expected pattern for documentation
            assert "docs" in metadata.url or "document" in metadata.url.lower()

        # Validate comprehensive metadata
        comprehensive_result = service.comprehensive_metadata_validation(metadata_list)
        assert comprehensive_result["overall_valid"] is True

    def test_chunk_id_uniquely_identifies_segment(self):
        """Test that chunk_id uniquely identifies the specific segment."""
        service = MetadataValidationService()

        # Create metadata with unique chunk_ids that identify specific segments
        metadata_list = [
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-intro-001",
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-arch-002",  # Different segment from same document
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-2/nodes-topics-services",
                chunk_id="services-concept-005",
                source_title="Chapter 2: Communication Primitives"
            )
        ]

        # Validate chunk_id uniqueness
        uniqueness_result = service.validate_chunk_id_uniqueness(metadata_list)

        assert uniqueness_result["all_unique"] is True
        assert uniqueness_result["duplicate_count"] == 0

        # Check that each chunk_id is meaningful and specific
        for metadata in metadata_list:
            assert metadata.chunk_id is not None
            assert metadata.chunk_id.strip() != ""
            # Verify chunk_id follows expected pattern (document-section-sequence)
            assert "-" in metadata.chunk_id  # Should have some structure

        # Validate comprehensive metadata
        comprehensive_result = service.comprehensive_metadata_validation(metadata_list)
        assert comprehensive_result["overall_valid"] is True

    def test_metadata_correctness_with_realistic_data(self):
        """Test metadata correctness with realistic book content data."""
        service = MetadataValidationService()

        # Simulate metadata from actual book content
        metadata_list = [
            Metadata(
                url="http://localhost:3002/docs/intro",
                chunk_id="intro-overview-001",
                source_title="Introduction to Physical AI and Humanoid Robotics"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-fundamentals-001",
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-2/nodes-topics-services",
                chunk_id="nodes-implementation-001",
                source_title="Chapter 2: Communication Primitives"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-2/nodes-topics-services",
                chunk_id="topics-implementation-002",  # Different chunk from same chapter
                source_title="Chapter 2: Communication Primitives"
            )
        ]

        # Run all validations
        completeness = service.validate_metadata_completeness(metadata_list)
        url_correctness = service.validate_url_correctness(metadata_list)
        uniqueness = service.validate_chunk_id_uniqueness(metadata_list)
        consistency = service.validate_metadata_consistency(metadata_list)
        accessibility = service.validate_metadata_accessibility(metadata_list)
        comprehensive = service.comprehensive_metadata_validation(metadata_list)

        # All validations should pass for correct metadata
        assert completeness["all_complete"] is True
        assert url_correctness["all_valid"] is True
        assert uniqueness["all_unique"] is True
        assert consistency["consistent"] is True
        assert accessibility["accessible"] is True
        assert comprehensive["overall_valid"] is True

        # Specific requirements from the spec:
        # "Each result includes complete metadata (URL, chunk_id, and other relevant attributes)"
        for metadata in metadata_list:
            assert metadata.url is not None and metadata.url.strip() != ""
            assert metadata.chunk_id is not None and metadata.chunk_id.strip() != ""
            assert metadata.source_title is not None

        # "URL correctly identifies the source document and chunk_id uniquely identifies the specific segment"
        url_to_chunks = {}
        for metadata in metadata_list:
            if metadata.url not in url_to_chunks:
                url_to_chunks[metadata.url] = []
            url_to_chunks[metadata.url].append(metadata.chunk_id)

        # Verify that URLs group related chunks and chunk_ids are unique within the set
        assert len(url_to_chunks) > 0  # We have multiple source documents
        assert uniqueness["duplicate_count"] == 0  # All chunk_ids are unique

    def test_metadata_correctness_requirement_validation(self):
        """Test the specific requirement: 'URL correctly identifies the source document and chunk_id uniquely identifies the specific segment'."""
        service = MetadataValidationService()

        # This test verifies the acceptance scenario:
        # "Given retrieved chunks with metadata, When examining the metadata fields,
        # Then the URL correctly identifies the source document and chunk_id uniquely identifies the specific segment"

        # Create metadata that satisfies the requirement
        valid_metadata = [
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-intro-001",
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-detailed-002",  # Different segment from same source
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-2/nodes-topics-services",
                chunk_id="nodes-basics-001",
                source_title="Chapter 2: Communication Primitives"
            )
        ]

        # Validate the requirement
        completeness_result = service.validate_metadata_completeness(valid_metadata)
        url_correctness_result = service.validate_url_correctness(valid_metadata)
        uniqueness_result = service.validate_chunk_id_uniqueness(valid_metadata)

        # Verify the requirement is met
        assert completeness_result["all_complete"] is True, "All metadata should be complete"
        assert url_correctness_result["all_valid"] is True, "All URLs should be valid and identify source documents"
        assert uniqueness_result["all_unique"] is True, "All chunk_ids should be unique"

        # Additional verification: group by URL to ensure chunk_ids are unique per document
        url_groups = {}
        for metadata in valid_metadata:
            if metadata.url not in url_groups:
                url_groups[metadata.url] = []
            url_groups[metadata.url].append(metadata.chunk_id)

        # Check that within each document group, chunk_ids are unique
        for url, chunk_ids in url_groups.items():
            assert len(chunk_ids) == len(set(chunk_ids)), f"Chunk IDs for {url} should be unique"

        # Verify each piece of metadata properly identifies both source and segment
        for metadata in valid_metadata:
            assert metadata.url and "http" in metadata.url, "URL should be valid and identify source"
            assert metadata.chunk_id and "-" in metadata.chunk_id, "Chunk ID should identify specific segment"

    def test_metadata_with_edge_cases(self):
        """Test metadata correctness with edge cases."""
        service = MetadataValidationService()

        # Test with metadata that might be on the boundary of correctness
        edge_case_metadata = [
            Metadata(
                url="http://localhost:3002/docs/very-long-title-with-many-hyphens-and-details",
                chunk_id="very-descriptive-chunk-id-with-lots-of-details-001",
                source_title="A Very Long and Descriptive Title for a Document"
            ),
            Metadata(
                url="https://example.com/simple",
                chunk_id="simple-001",
                source_title="Simple Doc"
            )
        ]

        # All validations should still pass
        result = service.comprehensive_metadata_validation(edge_case_metadata)

        assert result["overall_valid"] is True
        assert result["completeness"]["all_complete"] is True
        assert result["url_correctness"]["all_valid"] is True
        assert result["uniqueness"]["all_unique"] is True

    def test_metadata_identifies_correct_source_documents(self):
        """Test that metadata correctly identifies source documents for retrieval results."""
        service = MetadataValidationService()

        # Create metadata for results from a specific query about ROS concepts
        metadata_list = [
            Metadata(
                url="http://localhost:3002/docs/chapter-1/middleware-concept",
                chunk_id="middleware-definition-001",
                source_title="Chapter 1: ROS 2 as Middleware"
            ),
            Metadata(
                url="http://localhost:3002/docs/chapter-2/nodes-topics-services",
                chunk_id="nodes-definition-001",
                source_title="Chapter 2: Communication Primitives"
            )
        ]

        # Validate that metadata correctly identifies sources
        for metadata in metadata_list:
            # URL should identify a specific document
            assert metadata.url.startswith(("http://", "https://"))
            assert len(metadata.url) > 10  # Reasonable length for a document URL

            # Chunk ID should identify specific segment
            assert len(metadata.chunk_id) > 0
            assert metadata.chunk_id != metadata.url  # Should be different identifiers

            # Source title should provide context
            assert metadata.source_title is not None

        # Validate with service
        result = service.comprehensive_metadata_validation(metadata_list)
        assert result["overall_valid"] is True
        assert result["summary"]["validation_passed"] is True
        assert result["summary"]["completeness_percentage"] == 100.0