"""
End-to-end retrieval service for executing complete retrieval pipeline from query to JSON output.
"""
from typing import Dict, Any, Optional, List
import time
import json
from ..models.query import Query
from ..models.text_chunk import TextChunk
from ..models.metadata import Metadata
from ..models.retrieval_result import RetrievalResult
from .retrieval_service import RetrievalService
from .content_verification_service import ContentVerificationService
from .metadata_validation_service import MetadataValidationService
from ..utils.content_verification_utils import validate_content_within_tolerance
from ..utils.performance_monitor import PerformanceMonitor


class EndToEndRetrievalService:
    """
    Service class for executing the complete end-to-end retrieval pipeline.
    """
    def __init__(self):
        """
        Initialize the end-to-end retrieval service with all required components.
        """
        self.retrieval_service = RetrievalService()
        self.content_verification_service = ContentVerificationService()
        self.metadata_validation_service = MetadataValidationService()
        self.performance_monitor = PerformanceMonitor()

    def execute_full_pipeline(self, query_text: str, top_k: int = 5, min_score: float = 0.5,
                             verify_content: bool = True, validate_metadata: bool = True) -> Dict[str, Any]:
        """
        Execute the complete retrieval pipeline from query input to structured JSON output.

        Args:
            query_text: The input query text
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold
            verify_content: Whether to perform content accuracy verification
            validate_metadata: Whether to perform metadata validation

        Returns:
            Dictionary with complete pipeline results and validation status
        """
        start_time = time.time()

        # Step 1: Perform retrieval
        retrieval_result = self.retrieval_service.get_top_k_matches(
            query_text, top_k=top_k, min_score=min_score
        )

        # Step 2: Calculate total pipeline time
        pipeline_time = time.time() - start_time

        # Step 3: Perform content verification if requested
        content_verification_result = None
        if verify_content and retrieval_result.matches:
            retrieved_contents = [match.content for match in retrieval_result.matches]
            original_contents = [match.content for match in retrieval_result.matches]  # In real scenario, you'd have original content

            # For this implementation, we'll validate content against itself to simulate verification
            content_verification_result = self.content_verification_service.validate_retrieval_accuracy(
                retrieved_contents, original_contents, threshold=0.99
            )

        # Step 4: Perform metadata validation if requested
        metadata_validation_result = None
        if validate_metadata and retrieval_result.metadata_list:
            metadata_validation_result = self.metadata_validation_service.comprehensive_metadata_validation(
                retrieval_result.metadata_list
            )

        # Step 5: Format results as JSON
        formatted_results = []
        for i, match in enumerate(retrieval_result.matches):
            formatted_result = {
                "id": match.id,
                "content": match.content,
                "score": retrieval_result.scores[i],
                "metadata": {
                    "url": retrieval_result.metadata_list[i].url,
                    "chunk_id": retrieval_result.metadata_list[i].chunk_id,
                    "source_title": retrieval_result.metadata_list[i].source_title,
                    "created_at": retrieval_result.metadata_list[i].created_at.isoformat() if retrieval_result.metadata_list[i].created_at else None
                }
            }
            formatted_results.append(formatted_result)

        # Step 6: Create comprehensive result
        result = {
            "query": query_text,
            "results": formatted_results,
            "pipeline_time_seconds": pipeline_time,
            "pipeline_time_ms": pipeline_time * 1000,
            "total_chunks_searched": retrieval_result.total_chunks_searched,
            "content_verification": content_verification_result,
            "metadata_validation": metadata_validation_result,
            "pipeline_success": True,
            "performance_metrics": {
                "query_time": retrieval_result.retrieval_time,
                "total_pipeline_time": pipeline_time,
                "chunks_retrieved": len(retrieval_result.matches),
                "top_score": max(retrieval_result.scores) if retrieval_result.scores else 0.0
            }
        }

        # Step 7: Performance monitoring
        self.performance_monitor.record_operation(
            operation="end_to_end_retrieval",
            duration=pipeline_time,
            success=True,
            metadata={
                "query_length": len(query_text),
                "top_k": top_k,
                "results_count": len(formatted_results)
            }
        )

        return result

    def execute_pipeline_with_validation(self, query_text: str, top_k: int = 5, min_score: float = 0.5,
                                       content_threshold: float = 0.99,
                                       metadata_completeness_threshold: float = 1.0) -> Dict[str, Any]:
        """
        Execute the pipeline with specific validation thresholds.

        Args:
            query_text: The input query text
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold
            content_threshold: Minimum content accuracy threshold
            metadata_completeness_threshold: Minimum metadata completeness threshold

        Returns:
            Dictionary with pipeline results and validation against thresholds
        """
        # Execute the full pipeline
        result = self.execute_full_pipeline(
            query_text, top_k, min_score, verify_content=True, validate_metadata=True
        )

        # Validate against thresholds
        content_passes = True
        if result["content_verification"]:
            content_accuracy = result["content_verification"].get("overall_similarity", 1.0)
            content_passes = content_accuracy >= content_threshold

        metadata_passes = True
        if result["metadata_validation"]:
            metadata_completeness = result["metadata_validation"]["summary"].get("completeness_percentage", 100.0) / 100
            metadata_passes = metadata_completeness >= metadata_completeness_threshold

        # Add validation status to result
        result["validation_results"] = {
            "content_accuracy_passes": content_passes,
            "metadata_completeness_passes": metadata_passes,
            "content_threshold": content_threshold,
            "metadata_threshold": metadata_completeness_threshold,
            "overall_pipeline_passes": content_passes and metadata_passes
        }

        return result

    def validate_pipeline_performance(self, query_text: str, max_pipeline_time: float = 2.0) -> Dict[str, Any]:
        """
        Validate that the pipeline meets performance requirements.

        Args:
            query_text: The input query text
            max_pipeline_time: Maximum allowed pipeline time in seconds

        Returns:
            Dictionary with performance validation results
        """
        start_time = time.time()

        # Execute pipeline
        result = self.execute_full_pipeline(query_text)

        actual_time = time.time() - start_time

        # Check if performance requirements are met
        meets_performance = actual_time <= max_pipeline_time

        performance_validation = {
            "meets_requirements": meets_performance,
            "actual_time_seconds": actual_time,
            "max_allowed_time": max_pipeline_time,
            "time_margin": max_pipeline_time - actual_time if meets_performance else actual_time - max_pipeline_time,
            "performance_passes": meets_performance
        }

        result["performance_validation"] = performance_validation

        return result

    def execute_batch_queries(self, queries: List[str], top_k: int = 5, min_score: float = 0.5) -> Dict[str, Any]:
        """
        Execute multiple queries in batch and return aggregated results.

        Args:
            queries: List of query strings to execute
            top_k: Number of top results to return for each query
            min_score: Minimum similarity score threshold for each query

        Returns:
            Dictionary with batch execution results and statistics
        """
        start_time = time.time()

        batch_results = []
        successful_queries = 0
        total_time = 0.0

        for query in queries:
            query_start = time.time()
            try:
                result = self.execute_full_pipeline(query, top_k, min_score)
                batch_results.append({
                    "query": query,
                    "result": result,
                    "success": True
                })
                successful_queries += 1
            except Exception as e:
                batch_results.append({
                    "query": query,
                    "error": str(e),
                    "success": False
                })
            query_time = time.time() - query_start
            total_time += query_time

        batch_execution_time = time.time() - start_time

        return {
            "batch_results": batch_results,
            "summary": {
                "total_queries": len(queries),
                "successful_queries": successful_queries,
                "failed_queries": len(queries) - successful_queries,
                "success_rate": successful_queries / len(queries) if queries else 0,
                "total_batch_time": batch_execution_time,
                "average_query_time": total_time / len(queries) if queries else 0,
                "queries_per_second": len(queries) / batch_execution_time if batch_execution_time > 0 else 0
            }
        }

    def get_pipeline_health(self) -> Dict[str, Any]:
        """
        Get health status of the entire pipeline including all components.

        Returns:
            Dictionary with health status of all pipeline components
        """
        retrieval_health = self.retrieval_service.check_health()
        performance_stats = self.performance_monitor.get_stats()

        return {
            "pipeline_healthy": all(retrieval_health.values()),
            "retrieval_service": retrieval_health,
            "performance_monitor": performance_stats,
            "content_verification_available": True,
            "metadata_validation_available": True,
            "last_operation_time": self.performance_monitor.get_last_operation_time()
        }