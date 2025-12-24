"""
Utility functions for working with embeddings in the RAG retrieval system.
"""
import hashlib
from typing import List, Optional
import numpy as np
from ..models.vector_embedding import VectorEmbedding


def create_text_hash(text: str) -> str:
    """
    Create a hash of the input text for integrity verification.

    Args:
        text: The input text to hash

    Returns:
        A SHA-256 hash of the text
    """
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def normalize_vector(vector: List[float]) -> List[float]:
    """
    Normalize a vector to unit length.

    Args:
        vector: The input vector to normalize

    Returns:
        The normalized vector
    """
    if not vector:
        return vector

    np_vector = np.array(vector)
    norm = np.linalg.norm(np_vector)
    if norm == 0:
        return vector
    normalized = np_vector / norm
    return normalized.tolist()


def calculate_cosine_similarity(vec1: List[float], vec2: List[float]) -> float:
    """
    Calculate cosine similarity between two vectors.

    Args:
        vec1: First vector
        vec2: Second vector

    Returns:
        Cosine similarity score between -1 and 1
    """
    if not vec1 or not vec2 or len(vec1) != len(vec2):
        return 0.0

    np_vec1 = np.array(vec1)
    np_vec2 = np.array(vec2)

    dot_product = np.dot(np_vec1, np_vec2)
    norm1 = np.linalg.norm(np_vec1)
    norm2 = np.linalg.norm(np_vec2)

    if norm1 == 0 or norm2 == 0:
        return 0.0

    similarity = dot_product / (norm1 * norm2)
    return float(similarity)


def validate_embedding_dimensions(embeddings: List[VectorEmbedding]) -> bool:
    """
    Validate that all embeddings have the same dimension.

    Args:
        embeddings: List of embeddings to validate

    Returns:
        True if all embeddings have the same dimension, False otherwise
    """
    if not embeddings:
        return True

    expected_dimension = embeddings[0].dimension
    for embedding in embeddings:
        if embedding.dimension != expected_dimension:
            return False

    return True


def create_vector_embedding(vector: List[float], model: str, text: str) -> VectorEmbedding:
    """
    Create a VectorEmbedding object with validation.

    Args:
        vector: The embedding vector
        model: The model used to generate the embedding
        text: The original text for hashing

    Returns:
        A validated VectorEmbedding object
    """
    dimension = len(vector) if vector else 0
    text_hash = create_text_hash(text)

    embedding = VectorEmbedding(
        vector=vector,
        model=model,
        dimension=dimension,
        text_hash=text_hash
    )

    if not embedding.validate():
        raise ValueError("Invalid vector embedding parameters")

    return embedding