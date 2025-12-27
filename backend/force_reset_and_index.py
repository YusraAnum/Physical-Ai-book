import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent / "backend"
sys.path.insert(0, str(project_root))

# Force the settings to use OpenAI
os.environ['EMBEDDING_MODEL'] = 'openai'
os.environ['EMBEDDING_DIMENSION'] = '1536'
os.environ['OPENAI_API_KEY'] = 'dummy-key-for-testing'

# Reload the settings after setting environment variables
import importlib
from src.config import settings as settings_module
importlib.reload(settings_module)
from src.config.settings import settings

# Ensure settings are correct
settings.embedding_model = 'openai'
settings.embedding_dimension = 1536

from src.services.retrieval import RetrievalService
import uuid

def force_reset_and_index_documents():
    print(f"Settings: model={settings.embedding_model}, dimension={settings.embedding_dimension}")

    print("Initializing retrieval service...")
    retrieval_service = RetrievalService()

    # Delete the existing collection to fix dimension mismatch
    print("Deleting existing collection to fix dimension mismatch...")
    try:
        retrieval_service.client.delete_collection(settings.collection_name)
        print("Old collection deleted.")
    except Exception as e:
        print(f"Collection may not have existed, continuing: {e}")

    print("Ensuring collection exists with correct dimensions...")
    success = retrieval_service.ensure_collection_with_correct_dimensions()

    if success:
        print("Collection setup complete with correct dimensions!")

        # Check collection info
        collection_info = retrieval_service.get_collection_info()
        print(f"Collection info: {collection_info}")

        # Add test documents about ROS 2
        from qdrant_client.http import models

        test_documents = [
            {
                "id": str(uuid.uuid4()),
                "text": "ROS 2 (Robot Operating System 2) is flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS 2 is designed to be suitable for industrial applications and provides improved security, real-time support, and better architecture compared to ROS 1.",
                "document_id": "ros2-intro",
                "metadata": {
                    "title": "Introduction to ROS 2",
                    "page_reference": "Chapter 1, Page 1"
                }
            },
            {
                "id": str(uuid.uuid4()),
                "text": "ROS 2 architecture is built on DDS (Data Distribution Service) which provides a publish-subscribe communication pattern. This allows for better real-time performance and improved security compared to ROS 1's custom TCP/UDP implementation.",
                "document_id": "ros2-architecture",
                "metadata": {
                    "title": "ROS 2 Architecture",
                    "page_reference": "Chapter 2, Page 15"
                }
            },
            {
                "id": str(uuid.uuid4()),
                "text": "In ROS 2, packages are organized into workspaces using colcon build system. Nodes communicate through topics, services, and actions. The lifecycle of a node can be managed through the lifecycle node interface.",
                "document_id": "ros2-packages",
                "metadata": {
                    "title": "ROS 2 Packages and Nodes",
                    "page_reference": "Chapter 3, Page 28"
                }
            }
        ]

        # Upload documents to Qdrant
        points = []
        for doc in test_documents:
            # Generate embedding for the document
            embedding = retrieval_service._generate_embedding(doc["text"])
            print(f"Generated embedding of length: {len(embedding)} for document: {doc['metadata']['title']}")

            points.append(
                models.PointStruct(
                    id=doc["id"],
                    vector=embedding,
                    payload={
                        "text": doc["text"],
                        "document_id": doc["document_id"],
                        "metadata": doc["metadata"]
                    }
                )
            )

        # Upload all points at once
        retrieval_service.client.upsert(
            collection_name=settings.collection_name,
            points=points
        )

        print(f"{len(test_documents)} test documents indexed successfully!")

        # Verify indexing by searching
        print("\nTesting search functionality...")
        test_chunks = retrieval_service.search_chunks(
            query_text="What is ROS 2?",
            top_k=5,
            min_score=0.0  # Allow all results for testing
        )

        print(f"Search returned {len(test_chunks)} chunks:")
        for i, chunk in enumerate(test_chunks):
            print(f"  Chunk {i+1}: Score={chunk.similarity_score:.3f}, Content='{chunk.content[:100]}...'")

        print("\nIndexing completed successfully!")

    else:
        print("Failed to set up collection!")

if __name__ == "__main__":
    force_reset_and_index_documents()