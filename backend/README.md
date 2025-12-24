# RAG Agent Backend

A RAG-enabled backend agent for question answering over book content using OpenAI Agents SDK and Qdrant vector database.

## Overview

This service provides a REST API for querying book content using Retrieval-Augmented Generation (RAG). It integrates with Qdrant for vector search and OpenAI for response generation, ensuring answers are grounded in the provided content.

## Features

- Query full book content or specific document sections
- Semantic search using vector embeddings
- Context-aware responses from OpenAI models
- Proper attribution to source documents
- Safe handling of queries with no relevant content

## Prerequisites

- Python 3.11+
- OpenAI API key
- Qdrant Cloud account (or local instance)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>/backend
   ```

2. **Create a virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**

   Create a `.env` file in the backend directory with the following variables:

   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here  # Optional if using local instance
   COLLECTION_NAME=your_vector_collection_name
   OPENAI_MODEL=gpt-3.5-turbo  # Optional, defaults to gpt-3.5-turbo
   DEBUG=false  # Optional, set to true for debug mode
   ```

## Running the Service

### Development
```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### Production
```bash
cd backend
python main.py
```

The API will be available at `http://localhost:8000`

## API Endpoints

### Health Check
- `GET /health` - Check service health

### Chat Endpoints

#### Query Full Book Content
- `POST /api/v1/chat`
- Request body:
  ```json
  {
    "query": "Your question here",
    "user_id": "optional user identifier"
  }
  ```

#### Query Selected Text Only
- `POST /api/v1/chat/selected-text`
- Request body:
  ```json
  {
    "query": "Your question here",
    "document_ids": ["doc1", "doc2"],
    "user_id": "optional user identifier"
  }
  ```

### Response Format
Both endpoints return the same response format:
```json
{
  "response_id": "unique response identifier",
  "query_id": "original query identifier",
  "answer": "The generated answer",
  "sources": [
    {
      "chunk_id": "unique chunk identifier",
      "content": "The source content",
      "document_id": "source document identifier",
      "similarity_score": 0.85,
      "metadata": {}
    }
  ],
  "has_sources": true,
  "confidence": 0.85,
  "timestamp": "2025-12-22T10:30:00Z"
}
```

## Architecture

The service follows a clean architecture pattern with:

- **Models**: Pydantic models for data validation
- **Services**: Business logic encapsulation
  - `RetrievalService`: Qdrant integration and vector search
  - `AgentService`: OpenAI interaction and response generation
  - `RAGService`: Orchestration of retrieval and generation
- **API Layer**: FastAPI endpoints with proper routing

## Configuration

All configuration is handled through environment variables as defined in `src/config/settings.py`.

## Error Handling

The service handles various error scenarios:
- Invalid queries
- Missing document IDs
- Qdrant connection issues
- OpenAI API errors
- Empty retrieval results

## Testing

To run tests:
```bash
pip install -r requirements-dev.txt
pytest
```

## Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `OPENAI_API_KEY` | OpenAI API key | Yes |
| `QDRANT_URL` | Qdrant instance URL | Yes |
| `QDRANT_API_KEY` | Qdrant API key (if applicable) | No |
| `COLLECTION_NAME` | Qdrant collection name | No (defaults to "book-content") |
| `OPENAI_MODEL` | OpenAI model to use | No (defaults to "gpt-3.5-turbo") |
| `DEBUG` | Enable debug mode | No (defaults to false) |

## Performance

The service is designed to respond to queries within 10 seconds and handle up to 100 concurrent users.