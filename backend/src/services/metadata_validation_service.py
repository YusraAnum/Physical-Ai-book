"""
Metadata validation service for validating complete metadata in retrieved results.
"""
from typing import List, Dict, Any, Tuple
from ..models.metadata import Metadata


class MetadataValidationService:
    """
    Service class for validating the completeness and correctness of metadata in retrieval results.
    """

    def validate_metadata_completeness(self, metadata_list: List[Metadata]) -> Dict[str, Any]:
        """
        Validate that all required metadata fields are present in the results.

        Args:
            metadata_list: List of metadata objects to validate

        Returns:
            Dictionary with validation results
        """
        if not metadata_list:
            return {
                "all_complete": True,  # Vacuous truth: all 0 items are complete
                "total_count": 0,
                "complete_count": 0,
                "completeness_percentage": 100.0,  # All 0 items are complete (100%)
                "validation_details": []
            }

        validation_details = []
        complete_count = 0

        for i, metadata in enumerate(metadata_list):
            completeness = metadata.validate_completeness()
            is_complete = completeness["all_required_present"]

            validation_details.append({
                "index": i,
                "metadata_id": metadata.chunk_id,
                "completeness": completeness,
                "is_complete": is_complete
            })

            if is_complete:
                complete_count += 1

        total_count = len(metadata_list)
        completeness_percentage = (complete_count / total_count) * 100

        return {
            "all_complete": complete_count == total_count,
            "total_count": total_count,
            "complete_count": complete_count,
            "completeness_percentage": completeness_percentage,
            "validation_details": validation_details
        }

    def validate_url_correctness(self, metadata_list: List[Metadata]) -> Dict[str, Any]:
        """
        Validate that URLs in metadata correctly identify source documents.

        Args:
            metadata_list: List of metadata objects to validate

        Returns:
            Dictionary with URL validation results
        """
        if not metadata_list:
            return {
                "all_valid": True,
                "total_count": 0,
                "valid_count": 0,
                "valid_percentage": 0.0,
                "validation_details": []
            }

        validation_details = []
        valid_count = 0

        for i, metadata in enumerate(metadata_list):
            is_valid = metadata.validate()  # Uses the validate method from the Metadata model
            validation_details.append({
                "index": i,
                "chunk_id": metadata.chunk_id,
                "url": metadata.url,
                "is_valid": is_valid
            })

            if is_valid:
                valid_count += 1

        total_count = len(metadata_list)
        valid_percentage = (valid_count / total_count) * 100 if total_count > 0 else 0.0

        return {
            "all_valid": valid_count == total_count,
            "total_count": total_count,
            "valid_count": valid_count,
            "valid_percentage": valid_percentage,
            "validation_details": validation_details
        }

    def validate_chunk_id_uniqueness(self, metadata_list: List[Metadata]) -> Dict[str, Any]:
        """
        Validate that chunk_ids are unique across the result set.

        Args:
            metadata_list: List of metadata objects to validate

        Returns:
            Dictionary with chunk_id uniqueness validation results
        """
        if not metadata_list:
            return {
                "all_unique": True,
                "total_count": 0,
                "duplicate_count": 0,
                "unique_count": 0,
                "duplicates": []
            }

        chunk_ids = [metadata.chunk_id for metadata in metadata_list]
        unique_chunk_ids = set(chunk_ids)

        total_count = len(chunk_ids)
        unique_count = len(unique_chunk_ids)
        duplicate_count = total_count - unique_count

        # Find duplicate chunk_ids
        seen = set()
        duplicates = set()
        for chunk_id in chunk_ids:
            if chunk_id in seen:
                duplicates.add(chunk_id)
            else:
                seen.add(chunk_id)

        return {
            "all_unique": duplicate_count == 0,
            "total_count": total_count,
            "duplicate_count": duplicate_count,
            "unique_count": unique_count,
            "duplicates": list(duplicates)
        }

    def validate_metadata_consistency(self, metadata_list: List[Metadata]) -> Dict[str, Any]:
        """
        Validate consistency across metadata fields.

        Args:
            metadata_list: List of metadata objects to validate

        Returns:
            Dictionary with consistency validation results
        """
        if not metadata_list:
            return {
                "consistent": True,
                "issues": []
            }

        issues = []

        for i, metadata in enumerate(metadata_list):
            # Check for common issues
            if not metadata.url or not metadata.url.strip():
                issues.append({
                    "index": i,
                    "chunk_id": metadata.chunk_id,
                    "issue": "URL is empty or whitespace",
                    "field": "url"
                })

            if not metadata.chunk_id or not metadata.chunk_id.strip():
                issues.append({
                    "index": i,
                    "chunk_id": metadata.chunk_id,
                    "issue": "chunk_id is empty or whitespace",
                    "field": "chunk_id"
                })

            # Check for potentially invalid URLs
            if metadata.url and not metadata.url.startswith(('http://', 'https://')):
                issues.append({
                    "index": i,
                    "chunk_id": metadata.chunk_id,
                    "issue": "URL does not start with http:// or https://",
                    "field": "url"
                })

        return {
            "consistent": len(issues) == 0,
            "issues": issues
        }

    def validate_metadata_accessibility(self, metadata_list: List[Metadata], check_urls: bool = False) -> Dict[str, Any]:
        """
        Validate that metadata is accessible and properly formatted.

        Args:
            metadata_list: List of metadata objects to validate
            check_urls: Whether to perform URL accessibility checks (expensive operation)

        Returns:
            Dictionary with accessibility validation results
        """
        validation_results = {
            "accessible": True,
            "total_count": len(metadata_list),
            "accessible_count": 0,
            "accessibility_issues": []
        }

        accessible_count = 0

        for i, metadata in enumerate(metadata_list):
            # Check basic accessibility (non-null, properly formatted fields)
            has_url = bool(metadata.url and metadata.url.strip())
            has_chunk_id = bool(metadata.chunk_id and metadata.chunk_id.strip())

            is_accessible = has_url and has_chunk_id

            if is_accessible:
                accessible_count += 1
            else:
                validation_results["accessibility_issues"].append({
                    "index": i,
                    "chunk_id": metadata.chunk_id,
                    "url": metadata.url,
                    "missing_fields": {
                        "url": not has_url,
                        "chunk_id": not has_chunk_id
                    }
                })

        validation_results["accessible_count"] = accessible_count
        validation_results["accessible"] = accessible_count == len(metadata_list)
        validation_results["accessibility_percentage"] = (
            (accessible_count / len(metadata_list)) * 100 if metadata_list else 0.0
        )

        return validation_results

    def comprehensive_metadata_validation(self, metadata_list: List[Metadata]) -> Dict[str, Any]:
        """
        Perform comprehensive validation of metadata including completeness,
        correctness, uniqueness, and consistency.

        Args:
            metadata_list: List of metadata objects to validate

        Returns:
            Dictionary with comprehensive validation results
        """
        completeness_result = self.validate_metadata_completeness(metadata_list)
        url_correctness_result = self.validate_url_correctness(metadata_list)
        uniqueness_result = self.validate_chunk_id_uniqueness(metadata_list)
        consistency_result = self.validate_metadata_consistency(metadata_list)
        accessibility_result = self.validate_metadata_accessibility(metadata_list)

        # Overall validation status
        overall_valid = (
            completeness_result["all_complete"] and
            url_correctness_result["all_valid"] and
            uniqueness_result["all_unique"] and
            consistency_result["consistent"] and
            accessibility_result["accessible"]
        )

        return {
            "overall_valid": overall_valid,
            "completeness": completeness_result,
            "url_correctness": url_correctness_result,
            "uniqueness": uniqueness_result,
            "consistency": consistency_result,
            "accessibility": accessibility_result,
            "summary": {
                "total_metadata_count": len(metadata_list),
                "validation_passed": overall_valid,
                "completeness_percentage": completeness_result["completeness_percentage"],
                "url_validity_percentage": url_correctness_result["valid_percentage"],
                "accessibility_percentage": accessibility_result["accessibility_percentage"]
            }
        }