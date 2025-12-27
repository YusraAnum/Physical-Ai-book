from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from ..models.chunk import RetrievedChunk
from ..config.settings import settings
import logging

# Try to import cohere - it may not be installed
try:
    import cohere
    COHERE_AVAILABLE = True
except ImportError:
    COHERE_AVAILABLE = False
    cohere = None

logger = logging.getLogger(__name__)


class RetrievalService:
    """Qdrant retrieval service for finding relevant document chunks based on user queries"""

    def __init__(self):
        # Initialize Qdrant client
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            # If no API key, assume it's a local instance
            self.client = QdrantClient(
                url=settings.qdrant_url,
                timeout=10
            )

        # Initialize OpenAI client for embedding generation
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

        # Initialize Cohere client if API key is provided and library is available
        self.cohere_client = None
        if settings.cohere_api_key and COHERE_AVAILABLE:
            try:
                self.cohere_client = cohere.Client(settings.cohere_api_key)
            except Exception:
                logger.warning("Could not initialize Cohere client, falling back to OpenAI")
                self.cohere_client = None
        elif not COHERE_AVAILABLE:
            logger.warning("Cohere library not available, falling back to OpenAI")

        self.collection_name = settings.collection_name

    def _generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a text using the configured embedding API"""
        try:
            # Use the configured embedding model - prioritize settings over environment
            logger.info(f"Using embedding model: {settings.embedding_model}, dimension: {settings.embedding_dimension}")

            if settings.embedding_model.lower() == "cohere" and self.cohere_client:
                # Generate embedding using Cohere
                logger.info("Generating Cohere embedding")
                response = self.cohere_client.embed(
                    texts=[text],
                    model="embed-english-v3.0",  # Cohere's recommended embedding model
                    input_type="search_query"  # Specify this is a search query
                )
                embedding = response.embeddings[0]  # Return the first embedding
                logger.info(f"Generated Cohere embedding with length: {len(embedding)}")
                return embedding
            else:
                # Generate embedding using OpenAI - this should be the default
                logger.info("Generating OpenAI embedding")
                response = self.openai_client.embeddings.create(
                    input=text,
                    model="text-embedding-ada-002"  # Using OpenAI's embedding model (1536 dimensions)
                )
                embedding = response.data[0].embedding
                logger.info(f"Generated OpenAI embedding with length: {len(embedding)}")
                return embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def validate_document_filters(self, document_filters: Optional[List[str]]) -> bool:
        """Validate document filters to ensure they meet requirements"""
        if not document_filters:
            return True  # No filters is valid

        # Check that we have at least one filter
        if len(document_filters) == 0:
            return False

        # Check that each filter is a non-empty string
        for doc_filter in document_filters:
            if not doc_filter or not isinstance(doc_filter, str):
                return False

        # Optional: Check that the document IDs exist in the collection
        # For now, we'll just validate the format
        return True

    def search_chunks(self, query_text: str, top_k: int = 10, document_filters: Optional[List[str]] = None, min_score: Optional[float] = None) -> List[RetrievedChunk]:
        """Search for relevant chunks in Qdrant based on the query"""
        try:
            # Validate document filters
            if document_filters is not None and not self.validate_document_filters(document_filters):
                raise ValueError("Invalid document filters provided")

            # Generate embedding for the query
            query_embedding = self._generate_embedding(query_text)

            # Prepare filters if document filters are provided
            filters = None
            if document_filters:
                filters = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchAny(any=document_filters)
                        )
                    ]
                )

            # Try to ensure collection has correct dimensions before searching
            try:
                self.ensure_collection_with_correct_dimensions()
            except Exception as e:
                logger.warning(f"Could not ensure correct collection dimensions: {e}")

            # Perform search in Qdrant
            try:
                # In newer versions of qdrant-client, search method has been replaced with query_points
                if hasattr(self.client, 'query_points'):
                    # Use score threshold only if it's explicitly provided (not None)
                    score_threshold = min_score if min_score is not None else None
                    search_results = self.client.query_points(
                        collection_name=self.collection_name,
                        query=query_embedding,
                        query_filter=filters,
                        limit=top_k,
                        score_threshold=score_threshold,
                        with_payload=True,
                        with_vectors=False
                    )
                elif hasattr(self.client, 'search'):
                    # Fallback for older versions
                    score_threshold = min_score if min_score is not None else None
                    search_results = self.client.search(
                        collection_name=self.collection_name,
                        query_vector=query_embedding,
                        query_filter=filters,
                        limit=top_k,
                        score_threshold=score_threshold,
                        with_payload=True,
                        with_vectors=False
                    )
                else:
                    raise AttributeError("QdrantClient does not have 'query_points' or 'search' method. Please check qdrant-client version.")

                # Convert search results to RetrievedChunk objects
                # The structure of results may vary depending on the qdrant-client version
                retrieved_chunks = []

                # Handle different result structures depending on qdrant-client version
                if hasattr(search_results, 'points'):
                    # Newer version with query_points returns QueryResponse object
                    results_list = search_results.points
                else:
                    # Older version returns list directly
                    results_list = search_results

                for result in results_list:
                    # Handle different result structures depending on qdrant-client version
                    if hasattr(result, 'payload'):
                        # Newer version with query_points
                        payload = result.payload
                        chunk_id = result.id
                        similarity_score = result.score
                    elif isinstance(result, tuple):
                        # Older version might return tuples (point_id, vector, payload, score)
                        if len(result) >= 3:
                            chunk_id = result[0]
                            payload = result[2]  # payload is the third element
                            similarity_score = result[3] if len(result) > 3 else 0.0
                        else:
                            continue  # Skip if tuple doesn't have expected structure
                    else:
                        # Some other format
                        continue

                    # Create RetrievedChunk with safe access to payload fields
                    retrieved_chunk = RetrievedChunk(
                        chunk_id=chunk_id,
                        content=payload.get('text', '') if isinstance(payload, dict) else '',
                        document_id=payload.get('document_id', '') if isinstance(payload, dict) else '',
                        metadata=payload.get('metadata', {}) if isinstance(payload, dict) else {},
                        similarity_score=similarity_score
                    )
                    retrieved_chunks.append(retrieved_chunk)

                # Log retrieval results for debugging
                logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: '{query_text[:50]}...'")

                return retrieved_chunks
            except AttributeError as e:
                # Handle case where search method doesn't exist
                logger.error(f"QdrantClient search method not available: {e}")
                raise
            except Exception as e:
                # Handle case where Qdrant server is not running, collection doesn't exist, or dimension mismatch
                error_msg = str(e)
                logger.warning(f"Qdrant search failed: {error_msg}")

                # Check if it's a dimension mismatch error
                if "Vector dimension error" in error_msg or "expected dim:" in error_msg or "wrong vector dimensions" in error_msg.lower() or "dimension" in error_msg.lower():
                    logger.error(f"Vector dimension mismatch detected: {error_msg} - This indicates a collection embedding dimension mismatch")
                    # Raise the error instead of returning empty results to make the issue visible
                    raise

                # Return empty results instead of failing completely
                return []

        except Exception as e:
            logger.error(f"Error during chunk search: {e}")
            raise

    def check_collection_exists(self) -> bool:
        """Check if the collection exists in Qdrant"""
        try:
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]
            return self.collection_name in collection_names
        except Exception as e:
            logger.error(f"Error checking collection existence: {e}")
            return False

    def ensure_collection_with_correct_dimensions(self):
        """Ensure the collection exists with correct dimensions for OpenAI embeddings (1536)"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with correct dimensions (1536 for OpenAI ada-002)
                from qdrant_client.http.models import Distance, VectorParams
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
                )
                logger.info(f"Created new collection '{self.collection_name}' with 1536 dimensions")
                return True
            else:
                # Check existing collection's dimension
                collection_info = self.client.get_collection(self.collection_name)
                current_size = collection_info.config.params.vectors.size
                if current_size != 1536:
                    logger.warning(f"Collection has {current_size} dimensions, expected 1536. "
                                 f"This may cause embedding mismatches.")
                    # Instead of just warning, let's update the collection to the correct dimensions
                    # Note: This requires deleting and recreating the collection with the correct dimensions
                    # which would lose existing data. For now, we'll just return False to indicate mismatch.
                    # In a production environment, you might want to migrate the data.
                    return False
                return True
        except Exception as e:
            logger.error(f"Error ensuring collection with correct dimensions: {e}")
            return False

    def get_collection_info(self) -> dict:
        """Get information about the collection including point count"""
        try:
            if not self.check_collection_exists():
                logger.warning(f"Collection {self.collection_name} does not exist")
                return {"exists": False, "point_count": 0, "error": "Collection does not exist"}

            collection_info = self.client.get_collection(self.collection_name)
            return {
                "exists": True,
                "point_count": collection_info.points_count,
                "config": {
                    "vector_size": collection_info.config.params.vectors.size,
                    "distance": collection_info.config.params.vectors.distance
                }
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {"exists": False, "point_count": 0, "error": str(e)}