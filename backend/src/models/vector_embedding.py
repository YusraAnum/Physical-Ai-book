"""
VectorEmbedding utility representing numerical representation of text content that enables semantic similarity comparison.
"""
from dataclasses import dataclass
from typing import List


@dataclass
class VectorEmbedding:
    """
    Numerical representation of text content that enables semantic similarity comparison.

    Attributes:
        vector: The actual embedding vector
        model: Name of the model used to generate the embedding
        dimension: Number of dimensions in the vector
        text_hash: Hash of the original text for integrity verification
    """
    vector: List[float]
    model: str
    dimension: int
    text_hash: str

    def validate(self) -> bool:
        """
        Validate the vector embedding object.

        Returns:
            True if the vector embedding is valid, False otherwise
        """
        if not self.vector or len(self.vector) == 0:
            return False

        if not self.model or len(self.model.strip()) == 0:
            return False

        if self.dimension <= 0:
            return False

        if not self.text_hash or len(self.text_hash.strip()) == 0:
            return False

        # Check that vector length matches the declared dimension
        if len(self.vector) != self.dimension:
            return False

        return True