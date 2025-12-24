"""
Content verification service for validating that retrieved text chunks match original source content.
"""
from typing import Tuple, Dict, Any, Optional
import difflib
import hashlib
from ..utils.embedding_utils import calculate_cosine_similarity


class ContentVerificationService:
    """
    Service class for verifying the accuracy of retrieved content against original source content.
    """

    def verify_content_accuracy(self, retrieved_content: str, original_content: str, threshold: float = 0.99) -> Tuple[bool, float, Dict[str, Any]]:
        """
        Verify that retrieved content matches the original content within a threshold.

        Args:
            retrieved_content: The content retrieved from the vector database
            original_content: The original source content
            threshold: Minimum similarity threshold (0.0 to 1.0)

        Returns:
            Tuple of (is_accurate, similarity_score, details)
        """
        if not retrieved_content and not original_content:
            return True, 1.0, {"match_type": "both_empty"}

        if not retrieved_content or not original_content:
            return False, 0.0, {"match_type": "one_empty"}

        # Calculate text similarity using sequence matching
        similarity_score = difflib.SequenceMatcher(None, retrieved_content, original_content).ratio()

        # Check if similarity meets threshold
        is_accurate = similarity_score >= threshold

        # Create detailed results
        details = {
            "match_type": "text_similarity",
            "similarity_score": similarity_score,
            "threshold_used": threshold,
            "retrieved_length": len(retrieved_content),
            "original_length": len(original_content),
            "content_matches": is_accurate
        }

        return is_accurate, similarity_score, details

    def calculate_content_similarity(self, content1: str, content2: str) -> float:
        """
        Calculate similarity between two content strings.

        Args:
            content1: First content string
            content2: Second content string

        Returns:
            Similarity score between 0.0 and 1.0
        """
        if not content1 and not content2:
            return 1.0

        if not content1 or not content2:
            return 0.0

        return difflib.SequenceMatcher(None, content1, content2).ratio()

    def verify_content_integrity(self, retrieved_content: str, expected_hash: Optional[str] = None) -> Tuple[bool, str]:
        """
        Verify content integrity using hash comparison.

        Args:
            retrieved_content: The content to verify
            expected_hash: The expected hash of the original content

        Returns:
            Tuple of (is_integrity_valid, actual_hash)
        """
        actual_hash = hashlib.sha256(retrieved_content.encode('utf-8')).hexdigest()

        if expected_hash is None:
            return True, actual_hash

        is_valid = actual_hash == expected_hash
        return is_valid, actual_hash

    def compare_content_structures(self, retrieved_content: str, original_content: str) -> Dict[str, Any]:
        """
        Compare the structural elements of two content pieces.

        Args:
            retrieved_content: The retrieved content
            original_content: The original content

        Returns:
            Dictionary with structural comparison details
        """
        retrieved_lines = retrieved_content.splitlines()
        original_lines = original_content.splitlines()

        # Compare line counts
        line_count_match = len(retrieved_lines) == len(original_lines)

        # Compare word counts
        retrieved_words = len(retrieved_content.split())
        original_words = len(original_content.split())
        word_count_match = retrieved_words == original_words

        # Calculate line similarity
        line_similarity = 0.0
        if retrieved_lines and original_lines:
            min_len = min(len(retrieved_lines), len(original_lines))
            matching_lines = 0

            for i in range(min_len):
                if retrieved_lines[i] == original_lines[i]:
                    matching_lines += 1

            line_similarity = matching_lines / len(original_lines) if original_lines else 0.0

        return {
            "line_count_match": line_count_match,
            "word_count_match": word_count_match,
            "line_similarity": line_similarity,
            "retrieved_line_count": len(retrieved_lines),
            "original_line_count": len(original_lines),
            "retrieved_word_count": retrieved_words,
            "original_word_count": original_words
        }

    def get_content_differences(self, retrieved_content: str, original_content: str) -> Dict[str, Any]:
        """
        Get detailed differences between retrieved and original content.

        Args:
            retrieved_content: The retrieved content
            original_content: The original content

        Returns:
            Dictionary with detailed difference information
        """
        retrieved_lines = retrieved_content.splitlines(keepends=True)
        original_lines = original_content.splitlines(keepends=True)

        diff = list(difflib.unified_diff(
            original_lines,
            retrieved_lines,
            fromfile='original',
            tofile='retrieved',
            lineterm=''
        ))

        # Calculate statistics
        diff_ratio = difflib.SequenceMatcher(None, original_content, retrieved_content).ratio()
        similarity_percentage = round(diff_ratio * 100, 2)

        return {
            "differences": diff,
            "similarity_percentage": similarity_percentage,
            "total_differences": len(diff),
            "has_differences": len(diff) > 0
        }

    def validate_retrieval_accuracy(self, retrieved_chunks: list, original_chunks: list, threshold: float = 0.99) -> Dict[str, Any]:
        """
        Validate the accuracy of multiple retrieved chunks against original chunks.

        Args:
            retrieved_chunks: List of retrieved content chunks
            original_chunks: List of original content chunks
            threshold: Minimum similarity threshold

        Returns:
            Dictionary with validation results
        """
        if len(retrieved_chunks) != len(original_chunks):
            return {
                "accuracy_valid": False,
                "overall_similarity": 0.0,
                "chunk_count_match": False,
                "threshold": threshold,
                "valid_chunk_count": 0,
                "total_chunk_count": len(retrieved_chunks),
                "accuracy_percentage": 0.0,
                "details": "Chunk count mismatch"
            }

        total_similarity = 0.0
        valid_chunks = 0
        chunk_details = []

        for i, (retrieved, original) in enumerate(zip(retrieved_chunks, original_chunks)):
            is_accurate, similarity, detail = self.verify_content_accuracy(
                retrieved, original, threshold
            )

            chunk_details.append({
                "chunk_index": i,
                "is_accurate": is_accurate,
                "similarity": similarity,
                "detail": detail
            })

            total_similarity += similarity
            if is_accurate:
                valid_chunks += 1

        overall_similarity = total_similarity / len(retrieved_chunks) if retrieved_chunks else 0.0
        accuracy_valid = all(detail["is_accurate"] for detail in chunk_details)

        return {
            "accuracy_valid": accuracy_valid,
            "overall_similarity": overall_similarity,
            "chunk_count_match": len(retrieved_chunks) == len(original_chunks),
            "threshold": threshold,
            "valid_chunk_count": valid_chunks,
            "total_chunk_count": len(retrieved_chunks),
            "accuracy_percentage": (valid_chunks / len(retrieved_chunks)) * 100 if retrieved_chunks else 0.0,
            "details": chunk_details
        }