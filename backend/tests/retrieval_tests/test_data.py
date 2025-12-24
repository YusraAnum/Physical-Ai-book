"""
Test data fixtures for retrieval testing.
"""
from typing import List, Dict, Any
import numpy as np


def get_sample_text_chunks() -> List[Dict[str, Any]]:
    """
    Get sample text chunks for testing.

    Returns:
        List of sample text chunks with content, URL, and chunk ID
    """
    return [
        {
            "id": "chunk-1",
            "content": "ROS 2 uses a DDS-based middleware that provides reliable communication between nodes. This middleware layer handles message passing, service calls, and action communication in a distributed system.",
            "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
            "chunk_id": "middleware-intro-001"
        },
        {
            "id": "chunk-2",
            "content": "The middleware layer in ROS 2 handles communication protocols and data serialization. It provides a standardized way for different nodes to exchange information regardless of the programming language used.",
            "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
            "chunk_id": "middleware-impl-002"
        },
        {
            "id": "chunk-3",
            "content": "Nodes in ROS 2 are computational processes that perform specific tasks. They communicate with each other through topics, services, and actions using the underlying middleware.",
            "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
            "chunk_id": "nodes-concept-003"
        },
        {
            "id": "chunk-4",
            "content": "Topics in ROS 2 enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics.",
            "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
            "chunk_id": "topics-concept-004"
        },
        {
            "id": "chunk-5",
            "content": "Services in ROS 2 provide synchronous request-response communication between nodes. A client sends a request to a service server, which processes the request and returns a response.",
            "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
            "chunk_id": "services-concept-005"
        }
    ]


def get_sample_embeddings() -> List[List[float]]:
    """
    Get sample embeddings for testing (simulated).

    Returns:
        List of sample embedding vectors
    """
    # Generate deterministic but varied embeddings for testing
    embeddings = []
    for i in range(5):
        # Create a somewhat unique embedding for each chunk
        base_vector = [0.1 * (i + 1) for _ in range(1024)]
        # Add some variation to make them different
        for j in range(len(base_vector)):
            base_vector[j] += (j % 10) * 0.01
        embeddings.append(base_vector)

    return embeddings


def get_sample_query() -> str:
    """
    Get a sample query for testing.

    Returns:
        Sample query string
    """
    return "Explain ROS 2 middleware concepts"


def get_expected_results() -> List[Dict[str, Any]]:
    """
    Get expected results for the sample query.

    Returns:
        List of expected result dictionaries
    """
    return [
        {
            "id": "chunk-1",
            "score": 0.92,
            "content": "ROS 2 uses a DDS-based middleware that provides reliable communication between nodes. This middleware layer handles message passing, service calls, and action communication in a distributed system.",
            "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
            "chunk_id": "middleware-intro-001"
        },
        {
            "id": "chunk-2",
            "score": 0.87,
            "content": "The middleware layer in ROS 2 handles communication protocols and data serialization. It provides a standardized way for different nodes to exchange information regardless of the programming language used.",
            "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
            "chunk_id": "middleware-impl-002"
        }
    ]