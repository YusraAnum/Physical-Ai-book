"""
Qdrant client wrapper for interacting with the Qdrant vector database.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import os
from dotenv import load_dotenv
from ..models.text_chunk import TextChunk
from ..models.metadata import Metadata

# Load environment variables
load_dotenv()


class QdrantClientWrapper:
    """
    Wrapper class for Qdrant client to handle vector database operations.
    """
    def __init__(self):
        """
        Initialize the Qdrant client wrapper with connection parameters from environment.
        Falls back to local instance if remote connection fails.
        """
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "rag-embeddings")

        # Try to connect to remote Qdrant first
        if self.qdrant_url and self.qdrant_api_key:
            try:
                self.client = QdrantClient(
                    url=self.qdrant_url,
                    api_key=self.qdrant_api_key
                )
                # Test the connection
                self.client.get_collections()
                print(f"Connected to remote Qdrant: {self.qdrant_url}")
            except Exception as e:
                print(f"Failed to connect to remote Qdrant: {e}")
                print("Falling back to local Qdrant instance")
                self.client = QdrantClient(host="localhost", port=6333)
        else:
            # Use local instance if no remote credentials provided
            print("Using local Qdrant instance (localhost:6333)")
            self.client = QdrantClient(host="localhost", port=6333)

    def create_collection(self, vector_size: int = 1024, distance: Distance = Distance.COSINE):
        """
        Create a collection in Qdrant if it doesn't exist.

        Args:
            vector_size: Size of the vectors to be stored
            distance: Distance metric for similarity search
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=distance),
            )
            print(f"Created collection {self.collection_name}")

    def upsert_vectors(self, points: List[Dict[str, Any]]):
        """
        Upsert vectors into the Qdrant collection.

        Args:
            points: List of points to upsert, each with id, vector, and payload
        """
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search_vectors(self, query_vector: List[float], top_k: int = 5, filters: Optional[models.Filter] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the Qdrant collection.

        Args:
            query_vector: The query vector to search for
            top_k: Number of top results to return
            filters: Optional filters to apply to the search

        Returns:
            List of search results with payload and similarity scores
        """
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k,
            query_filter=filters,
            with_payload=True
        )

        results = []
        for hit in search_results:
            result = {
                "id": hit.id,
                "score": hit.score,
                "payload": hit.payload
            }
            results.append(result)

        return results

    def get_point(self, point_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific point from the collection by ID.

        Args:
            point_id: ID of the point to retrieve

        Returns:
            The point data or None if not found
        """
        points = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[point_id],
            with_payload=True
        )

        if points:
            point = points[0]
            return {
                "id": point.id,
                "payload": point.payload
            }

        return None

    def delete_collection(self):
        """
        Delete the collection from Qdrant.
        """
        try:
            self.client.delete_collection(self.collection_name)
            print(f"Deleted collection {self.collection_name}")
        except Exception as e:
            print(f"Error deleting collection {self.collection_name}: {e}")

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection information
        """
        collection_info = self.client.get_collection(self.collection_name)
        return {
            "name": collection_info.config.params.vectors.size,
            "vector_size": collection_info.config.params.vectors.size,
            "count": collection_info.points_count
        }

    def check_health(self) -> bool:
        """
        Check if the Qdrant connection is healthy.

        Returns:
            True if connection is healthy, False otherwise
        """
        try:
            self.client.get_collections()
            return True
        except:
            return False