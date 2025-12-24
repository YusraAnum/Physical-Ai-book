"""
Metadata model representing information that identifies the source document and specific chunk for traceability.
"""
from dataclasses import dataclass
from typing import Optional, Dict, Any
from datetime import datetime
import re


@dataclass
class Metadata:
    """
    Information that identifies the source document (URL) and specific chunk (chunk_id) for traceability.

    Attributes:
        url: URL of the source document
        chunk_id: Unique identifier for the specific chunk
        source_title: Title of the source document (optional)
        created_at: Timestamp when the metadata was created
        additional_metadata: Additional metadata fields as key-value pairs
    """
    url: str
    chunk_id: str
    source_title: Optional[str] = None
    created_at: datetime = None
    additional_metadata: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Initialize default values after object creation."""
        if self.created_at is None:
            self.created_at = datetime.now()

        if self.additional_metadata is None:
            self.additional_metadata = {}

    def validate(self) -> bool:
        """
        Validate the metadata object.

        Returns:
            True if the metadata is valid, False otherwise
        """
        if not self.url or len(self.url.strip()) == 0:
            return False

        # Basic URL validation
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        if not url_pattern.match(self.url):
            return False

        if not self.chunk_id or len(self.chunk_id.strip()) == 0:
            return False

        return True

    def validate_completeness(self) -> Dict[str, bool]:
        """
        Validate the completeness of metadata fields.

        Returns:
            Dictionary with validation status for each required field
        """
        completeness = {
            "url_present": bool(self.url and self.url.strip()),
            "chunk_id_present": bool(self.chunk_id and self.chunk_id.strip()),
            "source_title_present": bool(self.source_title and self.source_title.strip()),
            "created_at_present": self.created_at is not None,
            "all_required_present": True  # Will be updated below
        }

        # Check if all required fields are present
        completeness["all_required_present"] = (
            completeness["url_present"] and
            completeness["chunk_id_present"]
        )

        return completeness