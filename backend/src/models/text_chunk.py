"""
TextChunk model representing a segment of the original document that has been processed and stored as vector embeddings.
"""
from dataclasses import dataclass
from typing import List, Optional, Dict, Any


@dataclass
class TextChunk:
    """
    A segment of the original document that has been processed and stored as vector embeddings.

    Attributes:
        id: Unique identifier for the chunk
        content: The actual text content of the chunk
        original_url: URL of the source document
        vector: Vector embedding of the text content
        metadata: Additional metadata associated with the chunk
    """
    id: str
    content: str
    original_url: str
    vector: Optional[List[float]] = None
    metadata: Optional[Dict[str, Any]] = None

    def validate(self) -> bool:
        """
        Validate the text chunk object.

        Returns:
            True if the text chunk is valid, False otherwise
        """
        if not self.id or len(self.id.strip()) == 0:
            return False

        if not self.content or len(self.content.strip()) == 0:
            return False

        if not self.original_url or len(self.original_url.strip()) == 0:
            return False

        if self.vector is not None and len(self.vector) == 0:
            return False

        return True