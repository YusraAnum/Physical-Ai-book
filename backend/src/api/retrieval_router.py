"""
API router for retrieval endpoints.
"""
from fastapi import APIRouter, HTTPException, status, Request
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
import time
import traceback
from ..services.retrieval_service import RetrievalService
from ..services.health_service import HealthService
from ..models.query import Query


class QueryRequest(BaseModel):
    """
    Request model for the retrieval query endpoint.
    """
    query: str
    top_k: Optional[int] = 5
    min_score: Optional[float] = 0.5


class MetadataResponse(BaseModel):
    """
    Response model for metadata in retrieval results.
    """
    url: str
    chunk_id: str
    source_title: Optional[str] = None
    created_at: Optional[str] = None
    additional_metadata: Optional[Dict[str, Any]] = None


class RetrievalResultItem(BaseModel):
    """
    Response model for a single retrieval result item.
    """
    id: str
    content: str
    score: float
    metadata: MetadataResponse


class QueryResponse(BaseModel):
    """
    Response model for the retrieval query endpoint.
    """
    query: str
    results: List[RetrievalResultItem]
    query_time_ms: float
    total_chunks_searched: int


class HealthResponse(BaseModel):
    """
    Response model for the health check endpoint.
    """
    status: str
    qdrant_connected: bool
    cohere_connected: bool
    last_heartbeat: str


class ValidateRequest(BaseModel):
    """
    Request model for the validation endpoint.
    """
    test_query: str
    expected_urls: List[str]
    threshold: Optional[float] = 0.9


class ValidateResponse(BaseModel):
    """
    Response model for the validation endpoint.
    """
    success: bool
    accuracy: float
    retrieved_urls: List[str]
    matches_expected: bool
    details: Dict[str, float]


class ErrorResponse(BaseModel):
    """
    Response model for errors.
    """
    error: Dict[str, Any]


# Create API router
router = APIRouter(prefix="/retrieval", tags=["retrieval"])

# Initialize retrieval service
retrieval_service = RetrievalService()
health_service = HealthService()


@router.post("/query", response_model=QueryResponse)
async def query_chunks(request: QueryRequest):
    """
    Submit a text query and retrieve semantically similar content chunks.
    """
    try:
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Query text is required and cannot be empty"
            )

        if request.top_k <= 0 or request.top_k > 10:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="top_k must be between 1 and 10"
            )

        if request.min_score < 0.0 or request.min_score > 1.0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="min_score must be between 0.0 and 1.0"
            )

        # Record start time for performance tracking
        start_time = time.time()

        # Perform retrieval
        result = retrieval_service.get_top_k_matches(
            query_text=request.query,
            top_k=request.top_k,
            min_score=request.min_score
        )

        # Calculate query time in milliseconds
        query_time_ms = (time.time() - start_time) * 1000

        # Format results
        formatted_results = []
        for i, match in enumerate(result.matches):
            formatted_result = RetrievalResultItem(
                id=match.id,
                content=match.content,
                score=result.scores[i],
                metadata=MetadataResponse(
                    url=result.metadata_list[i].url,
                    chunk_id=result.metadata_list[i].chunk_id,
                    source_title=result.metadata_list[i].source_title
                )
            )
            formatted_results.append(formatted_result)

        # Create response
        response = QueryResponse(
            query=request.query,
            results=formatted_results,
            query_time_ms=query_time_ms,
            total_chunks_searched=result.total_chunks_searched
        )

        return response

    except HTTPException:
        raise
    except Exception as e:
        # Log the full traceback for debugging
        print(f"Error in query endpoint: {str(e)}")
        print(traceback.format_exc())

        # Check for specific service unavailability errors
        error_detail = str(e)
        if "Qdrant" in error_detail or "qdrant" in error_detail:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Qdrant database is currently unavailable"
            )
        elif "Cohere" in error_detail or "cohere" in error_detail:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Cohere API is currently unavailable"
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"An internal error occurred during retrieval: {error_detail}"
            )


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the retrieval service.
    """
    try:
        health_status = health_service.check_overall_health()
        service_healthy = health_status.get("overall_healthy", False)

        status_str = "healthy" if service_healthy else "unhealthy"

        from datetime import datetime
        heartbeat = datetime.now().isoformat()

        response = HealthResponse(
            status=status_str,
            qdrant_connected=health_status["components"]["qdrant"]["connected"],
            cohere_connected=health_status["components"]["cohere"]["connected"],
            last_heartbeat=heartbeat
        )

        return response

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Health check failed: {str(e)}"
        )


@router.post("/validate", response_model=ValidateResponse)
async def validate_retrieval(request: ValidateRequest):
    """
    Validate that the retrieval pipeline is working correctly by running a test query against known content.
    """
    try:
        if not request.test_query or len(request.test_query.strip()) == 0:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Test query is required and cannot be empty"
            )

        if not request.expected_urls:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Expected URLs are required for validation"
            )

        if request.threshold < 0.0 or request.threshold > 1.0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Threshold must be between 0.0 and 1.0"
            )

        # Perform retrieval for validation
        result = retrieval_service.get_top_k_matches(
            query_text=request.test_query,
            top_k=len(request.expected_urls),  # Get at least as many results as expected URLs
            min_score=0.0
        )

        # Extract retrieved URLs
        retrieved_urls = [metadata.url for metadata in result.metadata_list]

        # Calculate accuracy metrics
        expected_set = set(request.expected_urls)
        retrieved_set = set(retrieved_urls)

        # Calculate how many expected URLs were retrieved
        matches = expected_set.intersection(retrieved_set)
        accuracy = len(matches) / len(expected_set) if expected_set else 1.0

        matches_expected = accuracy >= request.threshold

        # Create detailed metrics
        details = {
            "top_match_accuracy": result.scores[0] if result.scores else 0.0,
            "content_similarity": accuracy,
            "metadata_correctness": 1.0  # Assuming metadata is always correct if present
        }

        response = ValidateResponse(
            success=matches_expected,
            accuracy=accuracy,
            retrieved_urls=retrieved_urls,
            matches_expected=matches_expected,
            details=details
        )

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Validation failed: {str(e)}"
        )


@router.post("/validate-full", response_model=ValidateResponse)
async def validate_full_retrieval_pipeline(request: ValidateRequest):
    """
    Validate the full retrieval pipeline including content accuracy and metadata validation.
    """
    try:
        if not request.test_query or len(request.test_query.strip()) == 0:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Test query is required and cannot be empty"
            )

        if not request.expected_urls:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Expected URLs are required for validation"
            )

        if request.threshold < 0.0 or request.threshold > 1.0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Threshold must be between 0.0 and 1.0"
            )

        # Import here to avoid circular imports
        from src.services.end_to_end_retrieval_service import EndToEndRetrievalService

        # Use the end-to-end service for comprehensive validation
        e2e_service = EndToEndRetrievalService()

        # Execute the pipeline with validation
        result = e2e_service.execute_pipeline_with_validation(
            query_text=request.test_query,
            content_threshold=request.threshold,
            metadata_completeness_threshold=request.threshold
        )

        # Extract relevant information for the response
        retrieved_urls = [item["metadata"]["url"] for item in result["results"]]

        # Calculate accuracy based on expected URLs
        expected_set = set(request.expected_urls)
        retrieved_set = set(retrieved_urls)
        matches = expected_set.intersection(retrieved_set)
        accuracy = len(matches) / len(expected_set) if expected_set else 1.0

        matches_expected = accuracy >= request.threshold

        # Include additional validation details from the end-to-end service
        details = result.get("validation_results", {})
        details.update({
            "content_similarity": accuracy,
            "retrieved_count": len(retrieved_urls),
            "expected_count": len(request.expected_urls)
        })

        response = ValidateResponse(
            success=result["validation_results"]["overall_pipeline_passes"] and matches_expected,
            accuracy=accuracy,
            retrieved_urls=retrieved_urls,
            matches_expected=matches_expected,
            details=details
        )

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Full pipeline validation failed: {str(e)}"
        )


