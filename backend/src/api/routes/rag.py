from fastapi import APIRouter, HTTPException
from typing import List, Optional
from pydantic import BaseModel, Field
from ...models.query import QueryRequest as OriginalQueryRequest, SelectedTextQueryRequest
from ...models.response import APIResponse as OriginalAPIResponse, SourceChunk as OriginalSourceChunk
from ...services.rag import RAGService
from ...services.retrieval import RetrievalService
from ...services.agent import AgentService
import logging
import uuid
import datetime

logger = logging.getLogger(__name__)

# Define models that match the API contract
class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    context: Optional[str] = Field(None, min_length=1, max_length=5000)
    session_id: Optional[str] = None

class Source(BaseModel):
    id: str
    title: str
    content: str
    page_reference: str

class APIResponse(BaseModel):
    response: str
    sources: List[Source]
    status: str
    session_id: str
    error_message: Optional[str] = None

class HealthResponse(BaseModel):
    status: str
    service: str
    timestamp: str

# Create a new router for RAG endpoints with no prefix (so endpoints are at /api/rag/*)
router = APIRouter()
rag_service = RAGService()
retrieval_service = RetrievalService()
agent_service = AgentService()

@router.post("/rag/query", response_model=APIResponse)
async def query_endpoint(request: QueryRequest):
    """Submit a query to the RAG system and receive a response"""
    try:
        # Validate the request
        if not request.query or len(request.query) < 1 or len(request.query) > 1000:
            raise HTTPException(status_code=422, detail="Query must be between 1 and 1000 characters")

        if request.context and (len(request.context) < 1 or len(request.context) > 5000):
            raise HTTPException(status_code=422, detail="Context must be between 1 and 5000 characters if provided")

        # Generate or validate session ID
        session_id = request.session_id or str(uuid.uuid4())

        # Validate session ID format if provided
        if request.session_id:
            try:
                uuid.UUID(request.session_id)
                session_id = request.session_id
            except ValueError:
                raise HTTPException(status_code=422, detail="Session ID must be a valid UUID if provided")

        # Process the query using the RAG service
        # Use the rag_service to process the query with context if provided
        agent_response = rag_service.query_full_content(
            query_text=request.query,
            user_id=None,  # No user_id in this endpoint
            context=request.context  # Pass context if provided
        )

        # Convert AgentResponse to the expected APIResponse format
        sources = [
            {
                "id": source_chunk.chunk_id,
                "title": source_chunk.metadata.get('title', 'Book Content') if source_chunk.metadata else 'Book Content',
                "content": source_chunk.content,
                "page_reference": source_chunk.metadata.get('page_reference', 'Unknown') if source_chunk.metadata else 'Unknown'
            }
            for source_chunk in agent_response.source_chunks or []
        ]

        result = {
            "response": agent_response.answer,
            "sources": sources,
            "status": "SUCCESS" if agent_response.has_sources else "SUCCESS_NO_SOURCES",
            "session_id": session_id
        }

        return APIResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        return APIResponse(
            response="",
            sources=[],
            status="ERROR",
            session_id=request.session_id or str(uuid.uuid4()),
            error_message=str(e)
        )

@router.get("/rag/health", response_model=HealthResponse)
async def rag_health_check():
    """Health check endpoint for the RAG service"""
    return HealthResponse(
        status="healthy",
        service="RAG backend",
        timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat()
    )