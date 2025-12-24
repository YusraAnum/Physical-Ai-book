"""
Cohere client wrapper for generating embeddings using the Cohere API.
"""
import os
from typing import List
import cohere
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class CohereClientWrapper:
    """
    Wrapper class for Cohere client to handle embedding generation.
    """
    def __init__(self, model: str = "embed-multilingual-v3.0"):
        """
        Initialize the Cohere client wrapper with API key from environment.

        Args:
            model: The embedding model to use
        """
        self.api_key = os.getenv("COHERE_API_KEY")
        self.model = model

        if not self.api_key:
            raise ValueError("COHERE_API_KEY must be set in environment variables")

        self.client = cohere.Client(api_key=self.api_key)

    def embed_text(self, texts: List[str], input_type: str = "search_query") -> List[List[float]]:
        """
        Generate embeddings for the provided texts.

        Args:
            texts: List of texts to embed
            input_type: Type of input (search_query, search_document, etc.)

        Returns:
            List of embedding vectors
        """
        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type=input_type
        )

        return response.embeddings

    def embed_single_text(self, text: str, input_type: str = "search_query") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed
            input_type: Type of input (search_query, search_document, etc.)

        Returns:
            Embedding vector
        """
        embeddings = self.embed_text([text], input_type)
        return embeddings[0] if embeddings else []

    def get_model_info(self) -> dict:
        """
        Get information about the embedding model.

        Returns:
            Dictionary with model information
        """
        # Note: Cohere doesn't have a direct API to get model info
        # We'll return basic info based on the model name
        return {
            "model": self.model,
            "type": "text-embedding",
            "dimension": self.estimate_embedding_dimension()
        }

    def estimate_embedding_dimension(self) -> int:
        """
        Estimate the embedding dimension based on the model.

        Returns:
            Estimated dimension size
        """
        # Different Cohere models have different dimensions
        # embed-multilingual-v3.0 typically has 1024 dimensions
        if "v3" in self.model:
            return 1024
        elif "v2" in self.model:
            return 768
        else:
            # Default assumption
            return 1024

    def check_health(self) -> bool:
        """
        Check if the Cohere API connection is healthy by making a test call.

        Returns:
            True if connection is healthy, False otherwise
        """
        try:
            # Test with a simple embedding request
            test_embedding = self.embed_single_text("test")
            return len(test_embedding) > 0
        except Exception:
            return False