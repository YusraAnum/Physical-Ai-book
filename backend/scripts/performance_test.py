"""
Performance benchmarking script for the RAG retrieval pipeline.
"""
import time
import statistics
import asyncio
from concurrent.futures import ThreadPoolExecutor
import requests
import json
from typing import List, Dict, Any
import argparse
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.performance_monitor import PerformanceMonitor


def run_single_query_test(api_url: str, query_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Run a single query test and measure performance.

    Args:
        api_url: The API endpoint URL
        query_data: The query data to send

    Returns:
        Dictionary with performance metrics
    """
    start_time = time.time()

    try:
        response = requests.post(
            f"{api_url}/retrieval/query",
            json=query_data,
            headers={"Content-Type": "application/json"},
            timeout=30  # 30 second timeout
        )

        end_time = time.time()
        total_time = end_time - start_time

        result = {
            "success": response.status_code == 200,
            "status_code": response.status_code,
            "response_time": total_time,
            "response_size": len(response.content) if response.content else 0,
            "error": None
        }

        if response.status_code == 200:
            try:
                json_response = response.json()
                result["result_count"] = len(json_response.get("results", []))
            except:
                result["result_count"] = 0
        else:
            result["error"] = response.text[:200]  # First 200 chars of error

    except Exception as e:
        end_time = time.time()
        result = {
            "success": False,
            "status_code": None,
            "response_time": end_time - start_time,
            "response_size": 0,
            "error": str(e),
            "result_count": 0
        }

    return result


def run_concurrent_tests(api_url: str, query_data: Dict[str, Any], num_concurrent: int = 10) -> List[Dict[str, Any]]:
    """
    Run concurrent query tests to measure performance under load.

    Args:
        api_url: The API endpoint URL
        query_data: The query data to send
        num_concurrent: Number of concurrent requests to make

    Returns:
        List of performance results
    """
    print(f"Running {num_concurrent} concurrent queries...")

    with ThreadPoolExecutor(max_workers=num_concurrent) as executor:
        futures = [
            executor.submit(run_single_query_test, api_url, query_data)
            for _ in range(num_concurrent)
        ]

        results = [future.result() for future in futures]

    return results


def run_load_test(api_url: str, query_data: Dict[str, Any], duration_seconds: int = 60) -> List[Dict[str, Any]]:
    """
    Run a load test for a specified duration.

    Args:
        api_url: The API endpoint URL
        query_data: The query data to send
        duration_seconds: Duration of the test in seconds

    Returns:
        List of performance results
    """
    print(f"Running load test for {duration_seconds} seconds...")

    start_time = time.time()
    results = []

    while (time.time() - start_time) < duration_seconds:
        result = run_single_query_test(api_url, query_data)
        results.append(result)

        # Small delay to prevent overwhelming the server
        time.sleep(0.1)

    return results


def calculate_statistics(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Calculate performance statistics from test results.

    Args:
        results: List of test results

    Returns:
        Dictionary with calculated statistics
    """
    successful_results = [r for r in results if r["success"]]
    failed_results = [r for r in results if not r["success"]]

    stats = {
        "total_requests": len(results),
        "successful_requests": len(successful_results),
        "failed_requests": len(failed_results),
        "success_rate": len(successful_results) / len(results) if results else 0,
    }

    if successful_results:
        response_times = [r["response_time"] for r in successful_results]
        response_sizes = [r["response_size"] for r in successful_results]
        result_counts = [r["result_count"] for r in successful_results]

        stats.update({
            "response_time": {
                "min": min(response_times),
                "max": max(response_times),
                "avg": statistics.mean(response_times),
                "median": statistics.median(response_times),
                "p95": sorted(response_times)[int(0.95 * len(response_times))] if response_times else 0,
                "p99": sorted(response_times)[int(0.99 * len(response_times))] if response_times else 0,
            },
            "response_size": {
                "min": min(response_sizes) if response_sizes else 0,
                "max": max(response_sizes) if response_sizes else 0,
                "avg": statistics.mean(response_sizes) if response_sizes else 0,
            },
            "result_count": {
                "min": min(result_counts) if result_counts else 0,
                "max": max(result_counts) if result_counts else 0,
                "avg": statistics.mean(result_counts) if result_counts else 0,
            }
        })

    return stats


def print_statistics(stats: Dict[str, Any]):
    """
    Print formatted performance statistics.
    """
    print("\n" + "="*60)
    print("PERFORMANCE TEST RESULTS")
    print("="*60)

    print(f"Total Requests: {stats['total_requests']}")
    print(f"Successful Requests: {stats['successful_requests']}")
    print(f"Failed Requests: {stats['failed_requests']}")
    print(f"Success Rate: {stats['success_rate']:.2%}")

    if 'response_time' in stats:
        print(f"\nResponse Time Metrics:")
        print(f"  Min: {stats['response_time']['min']:.3f}s")
        print(f"  Max: {stats['response_time']['max']:.3f}s")
        print(f"  Avg: {stats['response_time']['avg']:.3f}s")
        print(f"  Median: {stats['response_time']['median']:.3f}s")
        print(f"  P95: {stats['response_time']['p95']:.3f}s")
        print(f"  P99: {stats['response_time']['p99']:.3f}s")

        print(f"\nResponse Size Metrics:")
        print(f"  Min: {stats['response_size']['min']} bytes")
        print(f"  Max: {stats['response_size']['max']} bytes")
        print(f"  Avg: {int(stats['response_size']['avg'])} bytes")

        print(f"\nResult Count Metrics:")
        print(f"  Min: {stats['result_count']['min']}")
        print(f"  Max: {stats['result_count']['max']}")
        print(f"  Avg: {stats['result_count']['avg']:.1f}")

    print("="*60)


def main():
    """
    Main function to run performance tests.
    """
    parser = argparse.ArgumentParser(description="Performance benchmarking for RAG retrieval API")
    parser.add_argument("--api-url", default="http://localhost:8000",
                       help="API base URL (default: http://localhost:8000)")
    parser.add_argument("--query", default="Explain machine learning concepts",
                       help="Query text to test (default: 'Explain machine learning concepts')")
    parser.add_argument("--top-k", type=int, default=5,
                       help="Number of results to request (default: 5)")
    parser.add_argument("--min-score", type=float, default=0.5,
                       help="Minimum score threshold (default: 0.5)")
    parser.add_argument("--concurrent", type=int, default=10,
                       help="Number of concurrent requests (default: 10)")
    parser.add_argument("--duration", type=int, default=30,
                       help="Duration of load test in seconds (default: 30)")
    parser.add_argument("--test-type", choices=["single", "concurrent", "load"],
                       default="concurrent",
                       help="Type of test to run (default: concurrent)")

    args = parser.parse_args()

    print(f"Starting performance test for RAG retrieval API")
    print(f"API URL: {args.api_url}")
    print(f"Test type: {args.test_type}")

    # Prepare query data
    query_data = {
        "query": args.query,
        "top_k": args.top_k,
        "min_score": args.min_score
    }

    print(f"Query: '{args.query}' (top_k={args.top_k}, min_score={args.min_score})")

    # Run the appropriate test
    if args.test_type == "single":
        print("\nRunning single query test...")
        results = [run_single_query_test(args.api_url, query_data)]
    elif args.test_type == "concurrent":
        print(f"\nRunning concurrent test with {args.concurrent} requests...")
        results = run_concurrent_tests(args.api_url, query_data, args.concurrent)
    elif args.test_type == "load":
        print(f"\nRunning load test for {args.duration} seconds...")
        results = run_load_test(args.api_url, query_data, args.duration)

    # Calculate and display statistics
    stats = calculate_statistics(results)
    print_statistics(stats)

    # Check if performance targets are met
    print("\nPERFORMANCE ANALYSIS:")
    if 'response_time' in stats:
        avg_time = stats['response_time']['avg']
        p95_time = stats['response_time']['p95']

        print(f"✓ Average response time: {avg_time:.3f}s")
        print(f"✓ P95 response time: {p95_time:.3f}s")

        # Check against common performance targets
        if avg_time <= 2.0:
            print("✓ Average response time meets 2s target")
        else:
            print("✗ Average response time exceeds 2s target")

        if p95_time <= 5.0:
            print("✓ P95 response time meets 5s target")
        else:
            print("✗ P95 response time exceeds 5s target")

    print(f"\nSuccess rate: {stats['success_rate']:.2%}")
    if stats['success_rate'] >= 0.95:
        print("✓ Success rate meets 95% target")
    else:
        print("✗ Success rate below 95% target")


if __name__ == "__main__":
    main()