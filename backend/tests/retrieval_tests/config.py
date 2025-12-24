"""
Test configuration for retrieval testing.
"""
from typing import Dict, Any


class TestConfig:
    """
    Configuration class for retrieval tests.
    """
    # Default test settings
    DEFAULT_TOP_K = 5
    DEFAULT_MIN_SCORE = 0.5
    DEFAULT_TIMEOUT = 10.0  # seconds

    # Accuracy thresholds
    CONTENT_ACCURACY_THRESHOLD = 0.99  # 99% match required
    SEMANTIC_RELEVANCE_THRESHOLD = 0.90  # 90% relevance required
    METADATA_ACCURACY_THRESHOLD = 1.00  # 100% accuracy required

    # Performance requirements
    MAX_RETRIEVAL_TIME = 2.0  # seconds for end-to-end
    MAX_QUERY_TIME = 0.2  # seconds for individual query

    # Test data paths
    SAMPLE_DATA_PATH = "tests/retrieval_tests/sample_data/"

    # Mock settings
    MOCK_COHERE_ENABLED = True
    MOCK_QDRANT_ENABLED = True

    @classmethod
    def get_default_settings(cls) -> Dict[str, Any]:
        """
        Get default test settings as a dictionary.

        Returns:
            Dictionary of default test settings
        """
        return {
            "top_k": cls.DEFAULT_TOP_K,
            "min_score": cls.DEFAULT_MIN_SCORE,
            "timeout": cls.DEFAULT_TIMEOUT,
            "content_accuracy_threshold": cls.CONTENT_ACCURACY_THRESHOLD,
            "semantic_relevance_threshold": cls.SEMANTIC_RELEVANCE_THRESHOLD,
            "metadata_accuracy_threshold": cls.METADATA_ACCURACY_THRESHOLD,
            "max_retrieval_time": cls.MAX_RETRIEVAL_TIME,
            "max_query_time": cls.MAX_QUERY_TIME,
            "mock_cohere_enabled": cls.MOCK_COHERE_ENABLED,
            "mock_qdrant_enabled": cls.MOCK_QDRANT_ENABLED
        }

    @classmethod
    def get_performance_requirements(cls) -> Dict[str, float]:
        """
        Get performance requirements for validation.

        Returns:
            Dictionary of performance requirements
        """
        return {
            "max_retrieval_time": cls.MAX_RETRIEVAL_TIME,
            "max_query_time": cls.MAX_QUERY_TIME,
            "content_accuracy_threshold": cls.CONTENT_ACCURACY_THRESHOLD,
            "semantic_relevance_threshold": cls.SEMANTIC_RELEVANCE_THRESHOLD
        }