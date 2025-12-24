import pytest
import os
from unittest.mock import Mock, patch
from backend.main import EmbeddingPipeline


def test_embedding_pipeline_initialization():
    """Test that the EmbeddingPipeline initializes correctly"""
    # Mock environment variables
    with patch.dict(os.environ, {
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'http://localhost:6333',
        'BASE_URL': 'https://example.com'
    }):
        # Mock the Qdrant client to avoid actual connection
        with patch('backend.main.QdrantClient') as mock_qdrant:
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance
            mock_qdrant_instance.get_collections.return_value = Mock(collections=[])

            # Mock the Cohere client
            with patch('backend.main.cohere.Client'):
                pipeline = EmbeddingPipeline()

                # Check that initialization values are set
                assert pipeline.base_url == 'https://example.com'
                assert pipeline.chunk_size == 1000  # default value
                assert pipeline.qdrant_collection_name == 'rag-embeddings'  # default value


def test_chunk_text_basic():
    """Test basic text chunking functionality"""
    with patch.dict(os.environ, {
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'http://localhost:6333',
        'BASE_URL': 'https://example.com'
    }):
        with patch('backend.main.QdrantClient') as mock_qdrant:
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance
            mock_qdrant_instance.get_collections.return_value = Mock(collections=[])

            with patch('backend.main.cohere.Client'):
                pipeline = EmbeddingPipeline()

                # Test with a simple text
                text = "This is the first sentence. This is the second sentence. " * 50  # Make it long enough to chunk
                chunks = pipeline.chunk_text(text)

                # Should have created multiple chunks
                assert len(chunks) > 0
                # Each chunk should have text and size
                for chunk in chunks:
                    assert 'text' in chunk
                    assert 'size' in chunk
                    assert len(chunk['text']) > 0
                    assert chunk['size'] > 0


def test_chunk_text_empty():
    """Test text chunking with empty text"""
    with patch.dict(os.environ, {
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'http://localhost:6333',
        'BASE_URL': 'https://example.com'
    }):
        with patch('backend.main.QdrantClient') as mock_qdrant:
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance
            mock_qdrant_instance.get_collections.return_value = Mock(collections=[])

            with patch('backend.main.cohere.Client'):
                pipeline = EmbeddingPipeline()

                # Test with empty text
                chunks = pipeline.chunk_text("")
                assert len(chunks) == 0


if __name__ == "__main__":
    pytest.main([__file__])