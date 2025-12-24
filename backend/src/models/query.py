from pydantic import BaseModel, Field
from typing import List, Optional
import uuid
from datetime import datetime


class Query(BaseModel):
    """A natural language question or request from the user that triggers the RAG process"""

    query_text: str = Field(..., description="The user's question or request text", min_length=1, max_length=2000)
    document_filters: Optional[List[str]] = Field(None, description="Specific document identifiers to limit retrieval scope (default: null for full content search)")
    user_id: Optional[str] = Field(None, description="Identifier for the user making the query (for potential future use)")
    query_id: Optional[str] = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the query (auto-generated if not provided)")


class QueryRequest(BaseModel):
    """Request model for the chat endpoint"""

    query: str = Field(..., description="The user's question or query text", min_length=1, max_length=2000)
    user_id: Optional[str] = Field(None, description="Optional user identifier")


class SelectedTextQueryRequest(BaseModel):
    """Request model for the selected-text chat endpoint"""

    query: str = Field(..., description="The user's question or query text", min_length=1, max_length=2000)
    document_ids: List[str] = Field(..., description="List of document or text segment identifiers to search within")
    user_id: Optional[str] = Field(None, description="Optional user identifier")