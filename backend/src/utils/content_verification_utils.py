"""
Utility functions for content verification in the RAG retrieval system.
"""
from typing import List, Tuple, Dict, Any, Optional
import re
import unicodedata
from ..services.content_verification_service import ContentVerificationService


def normalize_text(text: str) -> str:
    """
    Normalize text for comparison by removing extra whitespace and standardizing formatting.

    Args:
        text: The text to normalize

    Returns:
        Normalized text
    """
    if not text:
        return text

    # Normalize unicode characters
    text = unicodedata.normalize('NFKD', text)

    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)

    # Strip leading/trailing whitespace
    text = text.strip()

    return text


def extract_text_segments(text: str, segment_length: int = 100) -> List[str]:
    """
    Extract segments from text for granular comparison.

    Args:
        text: The text to segment
        segment_length: Length of each segment in characters

    Returns:
        List of text segments
    """
    if not text:
        return []

    segments = []
    for i in range(0, len(text), segment_length):
        segment = text[i:i + segment_length]
        segments.append(segment)

    return segments


def compare_text_segments(retrieved_segments: List[str], original_segments: List[str], threshold: float = 0.95) -> Dict[str, Any]:
    """
    Compare segments of text to identify localized differences.

    Args:
        retrieved_segments: List of retrieved text segments
        original_segments: List of original text segments
        threshold: Minimum similarity threshold for segments

    Returns:
        Dictionary with segment comparison results
    """
    service = ContentVerificationService()

    if len(retrieved_segments) != len(original_segments):
        return {
            "segments_match": False,
            "match_percentage": 0.0,
            "threshold": threshold,
            "segment_count_match": False
        }

    matching_segments = 0
    segment_details = []

    for i, (retrieved, original) in enumerate(zip(retrieved_segments, original_segments)):
        similarity = service.calculate_content_similarity(retrieved, original)
        is_matching = similarity >= threshold

        segment_details.append({
            "segment_index": i,
            "similarity": similarity,
            "is_matching": is_matching,
            "retrieved_length": len(retrieved),
            "original_length": len(original)
        })

        if is_matching:
            matching_segments += 1

    match_percentage = (matching_segments / len(retrieved_segments)) * 100 if retrieved_segments else 0.0

    return {
        "segments_match": matching_segments == len(retrieved_segments),
        "match_percentage": match_percentage,
        "threshold": threshold,
        "segment_count_match": len(retrieved_segments) == len(original_segments),
        "matching_segments": matching_segments,
        "total_segments": len(retrieved_segments),
        "details": segment_details
    }


def calculate_text_fingerprint(text: str) -> Dict[str, Any]:
    """
    Calculate a fingerprint of the text for quick comparison.

    Args:
        text: The text to fingerprint

    Returns:
        Dictionary with text fingerprint information
    """
    # Count characters
    char_count = len(text)

    # Count words
    words = text.split()
    word_count = len(words)

    # Count sentences
    sentences = re.split(r'[.!?]+', text)
    sentence_count = len([s for s in sentences if s.strip()])

    # Count unique words
    unique_words = set(word.lower() for word in words)
    unique_word_count = len(unique_words)

    # Calculate average word length
    avg_word_length = sum(len(word) for word in words) / word_count if word_count > 0 else 0

    # Count special characters
    special_chars = sum(1 for c in text if not c.isalnum() and not c.isspace())

    return {
        "char_count": char_count,
        "word_count": word_count,
        "sentence_count": sentence_count,
        "unique_word_count": unique_word_count,
        "avg_word_length": round(avg_word_length, 2),
        "special_char_count": special_chars,
        "word_density": round(word_count / char_count * 100, 2) if char_count > 0 else 0
    }


def compare_text_fingerprints(fp1: Dict[str, Any], fp2: Dict[str, Any], tolerance: float = 0.05) -> Dict[str, bool]:
    """
    Compare two text fingerprints to see if they're likely from the same source.

    Args:
        fp1: First text fingerprint
        fp2: Second text fingerprint
        tolerance: Tolerance for comparison (0.05 = 5% difference allowed)

    Returns:
        Dictionary with comparison results
    """
    comparison_results = {}

    for key in fp1:
        if key in fp2 and isinstance(fp1[key], (int, float)) and isinstance(fp2[key], (int, float)):
            val1, val2 = fp1[key], fp2[key]
            if val1 == 0 and val2 == 0:
                comparison_results[key] = True
            elif val1 == 0 or val2 == 0:
                comparison_results[key] = False
            else:
                # Calculate percentage difference
                diff_percentage = abs(val1 - val2) / max(val1, val2)
                comparison_results[key] = diff_percentage <= tolerance
        else:
            # For non-numeric values or missing keys, mark as different
            comparison_results[key] = fp1.get(key) == fp2.get(key)

    return comparison_results


def detect_content_modifications(retrieved: str, original: str) -> Dict[str, Any]:
    """
    Detect specific types of modifications between retrieved and original content.

    Args:
        retrieved: Retrieved content
        original: Original content

    Returns:
        Dictionary with modification detection results
    """
    modifications = {
        "truncated": False,
        "extended": False,
        "reordered": False,
        "paraphrased": False,
        "missing_sections": [],
        "added_sections": []
    }

    # Check for truncation (retrieved is shorter and original starts with retrieved)
    if len(retrieved) < len(original) and original.startswith(retrieved):
        modifications["truncated"] = True

    # Check for extension (retrieved is longer and retrieved starts with original)
    if len(retrieved) > len(original) and retrieved.startswith(original):
        modifications["extended"] = True

    # Check for significant reordering by comparing sentence sequences
    orig_sentences = [s.strip() for s in re.split(r'[.!?]+', original) if s.strip()]
    retrieved_sentences = [s.strip() for s in re.split(r'[.!?]+', retrieved) if s.strip()]

    if orig_sentences and retrieved_sentences:
        # Check if sentence order is preserved
        common_sentences = set(orig_sentences) & set(retrieved_sentences)
        if common_sentences:
            orig_order = [s for s in orig_sentences if s in common_sentences]
            retrieved_order = [s for s in retrieved_sentences if s in common_sentences]

            if orig_order != retrieved_order and len(common_sentences) > 1:
                modifications["reordered"] = True

    # Check for paraphrasing (similar meaning but different wording)
    service = ContentVerificationService()
    similarity = service.calculate_content_similarity(
        normalize_text(retrieved),
        normalize_text(original)
    )

    if similarity < 0.8 and similarity > 0.3:  # Low text similarity but not completely different
        modifications["paraphrased"] = True

    return modifications


def validate_content_within_tolerance(retrieved: str, original: str, tolerance_level: str = "strict") -> Tuple[bool, Dict[str, Any]]:
    """
    Validate content match with different tolerance levels.

    Args:
        retrieved: Retrieved content
        original: Original content
        tolerance_level: "strict", "moderate", or "lenient"

    Returns:
        Tuple of (is_valid, details)
    """
    service = ContentVerificationService()

    # Set thresholds based on tolerance level
    thresholds = {
        "strict": 0.99,
        "moderate": 0.95,
        "lenient": 0.90
    }
    threshold = thresholds.get(tolerance_level, 0.95)

    # Perform basic similarity check
    is_accurate, similarity, details = service.verify_content_accuracy(
        retrieved, original, threshold
    )

    # Additional checks based on tolerance
    modifications = detect_content_modifications(retrieved, original)

    # For moderate/lenient tolerance, allow certain types of modifications
    if tolerance_level in ["moderate", "lenient"]:
        # If content is nearly identical, consider it valid even with minor modifications
        if similarity > 0.98:
            is_accurate = True

    result_details = {
        "similarity": similarity,
        "threshold_used": threshold,
        "is_accurate": is_accurate,
        "modifications": modifications,
        "tolerance_level": tolerance_level,
        "basic_validation": details
    }

    return is_accurate, result_details