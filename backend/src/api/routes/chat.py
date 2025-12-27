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
    context: Optional[str] = Field(None, max_length=5000)
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
        # Generate or validate session ID
        session_id = request.session_id or str(uuid.uuid4())
        if request.session_id:
            try:
                uuid.UUID(request.session_id)
                session_id = request.session_id
            except ValueError:
                raise HTTPException(status_code=422, detail="Session ID must be a valid UUID if provided")

        # Process the query through the RAG service
        # If context is provided, it should be handled by the RAG service
        agent_response = rag_service.query_full_content(
            query_text=request.query,
            user_id=None,  # No user_id in QueryRequest model
            context=request.context  # Pass context if provided
        )

        # Convert AgentResponse to the expected APIResponse format for this endpoint
        sources = [
            Source(
                id=chunk.chunk_id,
                title=chunk.metadata.get('title', 'Book Content') if chunk.metadata else 'Book Content',
                content=chunk.content,
                page_reference=chunk.metadata.get('page_reference', 'Unknown') if chunk.metadata else 'Unknown'
            )
            for chunk in (agent_response.source_chunks or [])
        ]

        # Create proper APIResponse with correct fields for this endpoint
        api_response = APIResponse(
            response=agent_response.answer,
            sources=sources,
            status="SUCCESS" if (agent_response.has_sources and len(sources) > 0) else "NO_SOURCES_FOUND",
            session_id=session_id,
            error_message=None if agent_response.has_sources else "No relevant sources found to answer the query"
        )

        return api_response

    except HTTPException as he:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        # Log the full error for debugging
        logger.error(f"Error processing query: {e}", exc_info=True)
        # Return proper JSON error response that frontend can parse
        return APIResponse(
            response="",
            sources=[],
            status="ERROR",
            session_id=session_id,
            error_message=f"Failed to process query: {str(e)}"
        )


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
        logger.error(f"Error processing selected text query: {e}", exc_info=True)
        # Return proper JSON error response instead of HTTPException
        return OriginalAPIResponse(
            response_id=str(uuid.uuid4()),
            query_id=str(uuid.uuid4()),
            answer="",
            sources=[],
            has_sources=False,
            confidence=None,
            timestamp=datetime.now()
        )


@router.get("/rag/health", response_model=HealthResponse)
async def rag_health_check():
    """Health check endpoint for the RAG service"""
    return HealthResponse(
        status="healthy",
        service="RAG backend",
        timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat()
    )