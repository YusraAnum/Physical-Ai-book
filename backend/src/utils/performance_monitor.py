"""
Performance monitoring utilities for the RAG retrieval pipeline.
"""
import time
from typing import Dict, Any, Callable, Optional
from functools import wraps
import logging

logger = logging.getLogger(__name__)


class PerformanceMonitor:
    """
    Utility class for monitoring performance metrics in the RAG pipeline.
    """

    def __init__(self):
        self.metrics = {}

    def measure_execution_time(self, func_name: str = None):
        """
        Decorator to measure execution time of functions.
        """
        def decorator(func: Callable) -> Callable:
            nonlocal func_name
            if func_name is None:
                func_name = func.__name__

            @wraps(func)
            def wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = func(*args, **kwargs)
                    execution_time = time.time() - start_time

                    # Log the execution time
                    logger.info(f"{func_name} executed in {execution_time:.3f} seconds")

                    # Store in metrics
                    if func_name not in self.metrics:
                        self.metrics[func_name] = []
                    self.metrics[func_name].append(execution_time)

                    return result
                except Exception as e:
                    execution_time = time.time() - start_time
                    logger.error(f"{func_name} failed after {execution_time:.3f} seconds: {str(e)}")
                    raise
            return wrapper
        return decorator

    def get_average_execution_time(self, func_name: str) -> Optional[float]:
        """
        Get the average execution time for a function.
        """
        if func_name in self.metrics and self.metrics[func_name]:
            times = self.metrics[func_name]
            return sum(times) / len(times)
        return None

    def get_total_calls(self, func_name: str) -> int:
        """
        Get the total number of calls for a function.
        """
        return len(self.metrics.get(func_name, []))

    def get_statistics(self) -> Dict[str, Dict[str, Any]]:
        """
        Get comprehensive statistics for all monitored functions.
        """
        stats = {}
        for func_name, times in self.metrics.items():
            if times:
                stats[func_name] = {
                    'total_calls': len(times),
                    'average_time': sum(times) / len(times),
                    'min_time': min(times),
                    'max_time': max(times),
                    'total_time': sum(times)
                }
        return stats

    def reset_metrics(self):
        """
        Reset all performance metrics.
        """
        self.metrics = {}

    def record_operation(self, operation: str, duration: float, success: bool, metadata: dict = None):
        """
        Record a specific operation for performance monitoring.

        Args:
            operation: Name of the operation
            duration: Duration of the operation in seconds
            success: Whether the operation was successful
            metadata: Additional metadata about the operation
        """
        if operation not in self.metrics:
            self.metrics[operation] = []

        operation_record = {
            'duration': duration,
            'success': success,
            'timestamp': time.time(),
            'metadata': metadata or {}
        }

        self.metrics[operation].append(operation_record)

        # Log the operation
        logger.info(f"Operation {operation} completed in {duration:.3f}s, success: {success}")

    def get_stats(self) -> Dict[str, Any]:
        """
        Get general performance statistics.
        """
        stats = {}
        overall_stats = {
            'total_operations': 0,
            'successful_operations': 0,
            'failed_operations': 0,
            'total_duration': 0.0,
            'durations': [],
            'success_flags': []
        }

        for operation, records in self.metrics.items():
            if records and isinstance(records[0], dict):  # Operation records with metadata
                durations = [r['duration'] for r in records]
                successful_ops = [r for r in records if r['success']]

                operation_stats = {
                    'total_operations': len(records),
                    'successful_operations': len(successful_ops),
                    'failed_operations': len(records) - len(successful_ops),
                    'average_duration': sum(durations) / len(durations) if durations else 0,
                    'min_duration': min(durations) if durations else 0,
                    'max_duration': max(durations) if durations else 0,
                    'total_duration': sum(durations)
                }

                stats[operation] = operation_stats

                # Update overall stats
                overall_stats['total_operations'] += len(records)
                overall_stats['successful_operations'] += len(successful_ops)
                overall_stats['failed_operations'] += len(records) - len(successful_ops)
                overall_stats['total_duration'] += sum(durations)
                overall_stats['durations'].extend(durations)
                overall_stats['success_flags'].extend([r['success'] for r in records])

            elif records:  # Simple timing records
                operation_stats = {
                    'total_calls': len(records),
                    'average_time': sum(records) / len(records),
                    'min_time': min(records) if records else 0,
                    'max_time': max(records) if records else 0,
                    'total_time': sum(records)
                }

                stats[operation] = operation_stats

        # Add overall statistics if we have operation records with metadata
        if overall_stats['durations']:
            # Calculate p95 (95th percentile)
            sorted_durations = sorted(overall_stats['durations'])
            p95_index = int(0.95 * len(sorted_durations))
            if p95_index >= len(sorted_durations):
                p95_index = len(sorted_durations) - 1
            p95_duration = sorted_durations[p95_index] if sorted_durations else 0

            stats.update({
                'total_operations': overall_stats['total_operations'],
                'successful_operations': overall_stats['successful_operations'],
                'failed_operations': overall_stats['failed_operations'],
                'success_rate': (overall_stats['successful_operations'] / overall_stats['total_operations']) if overall_stats['total_operations'] > 0 else 0,
                'avg_duration': sum(overall_stats['durations']) / len(overall_stats['durations']) if overall_stats['durations'] else 0,
                'min_duration': min(overall_stats['durations']) if overall_stats['durations'] else 0,
                'max_duration': max(overall_stats['durations']) if overall_stats['durations'] else 0,
                'p95_duration': p95_duration
            })

        return stats

    def get_last_operation_time(self) -> Optional[float]:
        """
        Get the timestamp of the last recorded operation.
        """
        latest_time = None
        for records in self.metrics.values():
            for record in records:
                if isinstance(record, dict) and 'timestamp' in record:
                    if latest_time is None or record['timestamp'] > latest_time:
                        latest_time = record['timestamp']
        return latest_time


def monitor_pipeline_performance():
    """
    Context manager for monitoring overall pipeline performance.
    """
    class PipelineMonitor:
        def __init__(self):
            self.start_time = None
            self.performance_monitor = PerformanceMonitor()

        def __enter__(self):
            self.start_time = time.time()
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            total_time = time.time() - self.start_time
            logger.info(f"Pipeline completed in {total_time:.3f} seconds")

            # Log statistics if available
            stats = self.performance_monitor.get_statistics()
            if stats:
                logger.info("Performance statistics:")
                for func_name, func_stats in stats.items():
                    logger.info(f"  {func_name}: avg={func_stats['average_time']:.3f}s, "
                              f"calls={func_stats['total_calls']}, "
                              f"total={func_stats['total_time']:.3f}s")

    return PipelineMonitor()


# Global performance monitor instance
global_performance_monitor = PerformanceMonitor()