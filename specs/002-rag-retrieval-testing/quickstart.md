# Quickstart Guide: RAG Retrieval Testing

## Overview
This guide provides instructions for setting up and running the RAG retrieval testing functionality to verify that stored vectors in Qdrant can be retrieved accurately.

## Prerequisites
- Python 3.11+
- uv package manager
- Access to Cohere API (with valid API key)
- Access to Qdrant database (with valid URL and API key)
- Existing vector embeddings stored in Qdrant

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Backend Directory
```bash
cd backend
```

### 3. Install Dependencies
```bash
uv sync
# or if using pip
pip install -r requirements.txt
```

### 4. Configure Environment Variables
Create or update the `.env` file in the backend directory with the following:
```env
# Cohere API Configuration
COHERE_API_KEY="your-cohere-api-key"

# Qdrant Configuration
QDRANT_URL="your-qdrant-url"
QDRANT_API_KEY="your-qdrant-api-key"

# Pipeline Configuration
BASE_URL="http://localhost:3002"  # or your actual book URL
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
COHERE_MODEL=embed-multilingual-v3.0
QDRANT_COLLECTION_NAME=rag-embeddings
BATCH_SIZE=10
RATE_LIMIT_DELAY=0.1
CRAWL_DEPTH=3
```

## Running Retrieval Tests

### 1. Basic Retrieval Test
```bash
# Run the main retrieval test script
uv run main.py
```

This will execute the end-to-end retrieval pipeline and verify that stored vectors can be retrieved accurately.

### 2. Specific Test Functions
```bash
# Run specific retrieval tests
uv run -m pytest tests/retrieval_tests/ -v
```

### 3. Query Testing
To test specific queries against the vector database:

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models
import os
from dotenv import load_dotenv

load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Example query
query_text = "your query text here"

# Convert query to embedding using Cohere API
# (Implementation details would be in the actual code)
```

## Verification Steps

### 1. Top-K Matching Verification
- Submit a test query
- Verify that the system returns the top-K most semantically similar text chunks
- Check that results are ranked by relevance

### 2. Content Accuracy Verification
- Compare retrieved text chunks with original source content
- Verify that content matches with 99% accuracy
- Check for any content corruption or modification

### 3. Metadata Validation
- Verify that URL and chunk_id are correctly returned with each result
- Ensure metadata points to correct source documents
- Validate metadata integrity during retrieval

### 4. End-to-End Pipeline Test
- Execute complete query through the retrieval pipeline
- Verify clean JSON output format
- Check that response contains top-K matches, original text, and metadata

## Expected Output
The retrieval test should produce output similar to:
```
INFO - Starting retrieval test...
INFO - Query: "ROS 2 middleware concepts"
INFO - Retrieved 3 top matches:
INFO - Match 1: Score: 0.92, URL: http://localhost:3002/docs/chapter-1/middleware-concept
INFO - Match 2: Score: 0.87, URL: http://localhost:3002/docs/chapter-2/nodes-topics-services
INFO - Match 3: Score: 0.81, URL: http://localhost:3002/docs/intro
INFO - Content accuracy: 99.5% match with original
INFO - All metadata fields present and correct
INFO - Pipeline completed successfully in 1.2 seconds
```

## Troubleshooting

### Common Issues

1. **Qdrant Connection Errors**
   - Verify QDRANT_URL and QDRANT_API_KEY in .env file
   - Check that Qdrant service is running and accessible

2. **Cohere API Errors**
   - Verify COHERE_API_KEY is valid and has sufficient quota
   - Check internet connectivity to Cohere API

3. **No Results Returned**
   - Ensure vector embeddings exist in Qdrant collection
   - Verify collection name matches QDRANT_COLLECTION_NAME

4. **Performance Issues**
   - Check that query embedding model matches stored embedding model
   - Verify vector dimensions are consistent