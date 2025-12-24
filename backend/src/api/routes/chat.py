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

        # Validate session ID format if provided
        if request.session_id:
            try:
                uuid.UUID(request.session_id)
            except ValueError:
                raise HTTPException(status_code=422, detail="Session ID must be a valid UUID if provided")

        # Process the query through the RAG service
        # If context is provided, it should be handled by the RAG service
        agent_response = rag_service.query_full_content(
            query_text=request.query,
            user_id=None,  # No user_id in QueryRequest model
            context=request.context  # Pass context if provided
        )

        # Convert AgentResponse to APIResponse
        api_response = APIResponse(
            response_id=agent_response.response_id,
            query_id=agent_response.query_id,
            answer=agent_response.answer,
            sources=[
                {
                    "chunk_id": chunk.chunk_id,
                    "content": chunk.content,
                    "document_id": chunk.document_id,
                    "similarity_score": chunk.similarity_score,
                    "metadata": chunk.metadata
                }
                for chunk in (agent_response.source_chunks or [])
            ],
            has_sources=agent_response.has_sources,
            confidence=agent_response.confidence_level,
            timestamp=agent_response.timestamp
        )

        return api_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail="Internal server error processing query")


@router.post("/chat/selected-text", response_model=OriginalAPIResponse)
async def chat_selected_text(request: SelectedTextQueryRequest):
    """Query the RAG agent with selected text only"""
    try:
        # Validate that the Qdrant collection exists
        if not retrieval_service.check_collection_exists():
            raise HTTPException(status_code=500, detail="Qdrant collection does not exist")

        # Process the query through the RAG service
        agent_response = rag_service.query_selected_text(
            query_text=request.query,
            document_ids=request.document_ids,
            user_id=request.user_id
        )

        # Convert AgentResponse to APIResponse
        api_response = OriginalAPIResponse(
            response_id=agent_response.response_id,
            query_id=agent_response.query_id,
            answer=agent_response.answer,
            sources=[
                OriginalSourceChunk(
                    chunk_id=chunk.chunk_id,
                    content=chunk.content,
                    document_id=chunk.document_id,
                    similarity_score=chunk.similarity_score,
                    metadata=chunk.metadata
                )
                for chunk in (agent_response.source_chunks or [])
            ],
            has_sources=agent_response.has_sources,
            confidence=agent_response.confidence_level,
            timestamp=agent_response.timestamp
        )

        return api_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing selected text query: {e}")
        raise HTTPException(status_code=500, detail="Internal server error processing query")


@router.get("/rag/health", response_model=HealthResponse)
async def rag_health_check():
    """Health check endpoint for the RAG service"""
    return HealthResponse(
        status="healthy",
        service="RAG backend",
        timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat()
    )