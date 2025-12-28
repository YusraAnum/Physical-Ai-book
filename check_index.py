#!/usr/bin/env python3
"""
Script to check the current state of the vector database and verify document indexing.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.services.retrieval import RetrievalService
import logging

# Set up logging to see detailed output
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def check_vector_database():
    """Check the state of the vector database"""
    print("Checking vector database state...")

    try:
        # Initialize the retrieval service
        retrieval_service = RetrievalService()

        # Check if the collection exists
        collection_exists = retrieval_service.check_collection_exists()
        print(f"Collection exists: {collection_exists}")

        if collection_exists:
            # Get collection information
            collection_info = retrieval_service.get_collection_info()
            print(f"Collection info: {collection_info}")

            # Try a sample query to see if there are any documents
            print("\nTesting with a sample query...")
            try:
                # Perform a search with a general query
                sample_chunks = retrieval_service.search_chunks(
                    query_text="What is ROS 2?",
                    top_k=5
                )
                print(f"Number of chunks retrieved: {len(sample_chunks)}")

                if sample_chunks:
                    print("Sample retrieved chunks:")
                    for i, chunk in enumerate(sample_chunks[:3]):  # Show first 3 chunks
                        print(f"  Chunk {i+1}:")
                        print(f"    ID: {chunk.chunk_id}")
                        print(f"    Content preview: {chunk.content[:100]}...")
                        print(f"    Similarity score: {chunk.similarity_score}")
                        print(f"    Document ID: {chunk.document_id}")
                        print(f"    Metadata: {chunk.metadata}")
                        print()
                else:
                    print("No chunks were retrieved - the database might be empty or there could be an issue.")
            except Exception as e:
                print(f"Error during sample query: {e}")
                import traceback
                traceback.print_exc()
        else:
            print("Collection does not exist - no documents are indexed yet.")

    except Exception as e:
        print(f"Error initializing retrieval service: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_vector_database()