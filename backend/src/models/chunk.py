from pydantic import BaseModel, Field
from typing import Dict, Optional


class RetrievedChunk(BaseModel):
    """Document segments retrieved from Qdrant vector database that are relevant to the user's query"""

    chunk_id: str = Field(..., description="Unique identifier for the document chunk")
    content: str = Field(..., description="The actual text content of the chunk")
    document_id: str = Field(..., description="Identifier of the source document")
    metadata: Optional[Dict] = Field(None, description="Additional metadata about the chunk (source URL, page number, etc.)")
    similarity_score: float = Field(..., description="Similarity score from the vector search (-1.0 to 1.0)", ge=-1.0, le=1.0)