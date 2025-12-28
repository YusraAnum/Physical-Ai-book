---
title: RAG Agent Backend API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
app_port: 7860
---

# RAG Agent Backend API

A RAG-enabled backend agent for question answering over book content using OpenAI Agents SDK and Qdrant vector database.

## Features

- Query full book content or specific document sections
- Semantic search using vector embeddings
- Context-aware responses from OpenAI models
- Proper attribution to source documents
- Safe handling of queries with no relevant content

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

## Configuration

Set the following secrets in your Hugging Face Space settings:

- `OPENAI_API_KEY` - Your OpenAI API key
- `QDRANT_URL` - Your Qdrant Cloud URL
- `QDRANT_API_KEY` - Your Qdrant API key
- `COLLECTION_NAME` - Qdrant collection name (optional, defaults to "book-content")
- `OPENAI_MODEL` - OpenAI model to use (optional, defaults to "gpt-3.5-turbo")

## Usage

Once deployed, you can access the API documentation at:
- Swagger UI: `https://your-space.hf.space/docs`
- ReDoc: `https://your-space.hf.space/redoc`

## Example Request

```bash
curl -X POST "https://your-space.hf.space/api/v1/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is physical AI?",
    "user_id": "user123"
  }'
```

## Architecture

- **FastAPI** - Modern Python web framework
- **OpenAI** - LLM for response generation
- **Qdrant** - Vector database for semantic search
- **Pydantic** - Data validation and settings management
