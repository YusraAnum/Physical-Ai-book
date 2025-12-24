"""
Pytest configuration and fixtures for retrieval tests.
"""
import pytest
import os
from unittest.mock import Mock, patch
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables for testing
load_dotenv()

@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for testing."""
    with patch('qdrant_client.QdrantClient') as mock:
        mock_instance = Mock()
        mock.return_value = mock_instance
        yield mock_instance

@pytest.fixture
def mock_cohere_client():
    """Mock Cohere client for testing."""
    with patch('cohere.Client') as mock:
        mock_instance = Mock()
        mock.return_value = mock_instance
        yield mock_instance

@pytest.fixture
def sample_text_chunks():
    """Sample text chunks for testing."""
    return [
        {
            "id": "chunk-1",
            "content": "ROS 2 uses a DDS-based middleware that provides reliable communication between nodes.",
            "url": "http://localhost:3002/docs/chapter-1/middleware-concept",
            "chunk_id": "middleware-intro-001"
        },
        {
            "id": "chunk-2",
            "content": "The middleware layer in ROS 2 handles communication protocols and data serialization.",
            "url": "http://localhost:3002/docs/chapter-2/nodes-topics-services",
            "chunk_id": "middleware-impl-002"
        }
    ]

@pytest.fixture
def sample_query():
    """Sample query for testing."""
    return "Explain ROS 2 middleware concepts"