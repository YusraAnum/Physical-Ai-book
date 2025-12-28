#!/usr/bin/env python3
"""
Start a local Qdrant instance for development
"""
import os
import tempfile
from qdrant_client import QdrantClient

# Create a temporary directory for Qdrant storage
temp_dir = tempfile.mkdtemp(prefix="qdrant_")
print(f"Starting local Qdrant at: {temp_dir}")

# Initialize local Qdrant client
client = QdrantClient(path=temp_dir)  # This creates a local instance

print("Local Qdrant is running!")
print(f"Storage path: {temp_dir}")
print("Press Ctrl+C to stop")

# Keep the process alive
try:
    # Check if the client works
    collections = client.get_collections()
    print(f"Connected to local Qdrant. Current collections: {[c.name for c in collections.collections]}")

    # Keep the server running
    import time
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nShutting down local Qdrant...")
    exit(0)