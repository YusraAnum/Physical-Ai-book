"""
RetrievalResult model representing the combination of matched text chunks and their associated metadata returned to the user.
"""
from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
from .text_chunk import TextChunk
from .metadata import Metadata
from .query import Query


@dataclass
class RetrievalResult:
    """
    The combination of matched text chunks and their associated metadata returned to the user.

    Attributes:
        query: The original query that generated this result
        matches: Top-K most similar text chunks
        scores: Similarity scores for each match
        metadata_list: Metadata for each matched chunk
        retrieval_time: Time taken to retrieve the results
        total_chunks_searched: Total number of chunks searched
    """
    query: Query
    matches: List[TextChunk]
    scores: List[float]
    metadata_list: List[Metadata]
    retrieval_time: float
    total_chunks_searched: int

    def validate(self) -> bool:
        """
        Validate the retrieval result object.

        Returns:
            True if the retrieval result is valid, False otherwise
        """
        if not self.matches or len(self.matches) == 0:
            # Allow empty matches if no relevant results found
            return True

        # Check that matches and scores have the same length
        if len(self.matches) != len(self.scores):
            return False

        # Check that matches and metadata_list have the same length
        if len(self.matches) != len(self.metadata_list):
            return False

        # Check that scores are in descending order (most relevant first)
        for i in range(len(self.scores) - 1):
            if self.scores[i] < self.scores[i + 1]:
                return False

        return True