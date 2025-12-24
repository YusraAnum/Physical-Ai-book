# Quickstart: RAG Agent Backend

## Overview
This guide will help you set up and run the RAG-enabled backend agent for question answering over book content.

## Prerequisites
- Python 3.11+
- pip package manager
- Access to OpenAI API (API key)
- Access to Qdrant Cloud (API key and URL)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r backend/requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COLLECTION_NAME=your_vector_collection_name
```

## Running the Service

### 1. Start the Development Server
```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Verify the Service is Running
Open your browser to `http://localhost:8000/docs` to view the API documentation.

## Making Requests

### Query Full Book Content
```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of humanoid robotics?"
  }'
```

### Query Selected Text Only
```bash
curl -X POST "http://localhost:8000/chat/selected-text" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does the book say about gait control?",
    "document_ids": ["chapter-3", "section-4.2"]
  }'
```

## Expected Response Format
```json
{
  "response_id": "resp-67890",
  "query_id": "query-12345",
  "answer": "The key principles of humanoid robotics include...",
  "sources": [
    {
      "chunk_id": "chunk-abc123",
      "content": "Humanoid robotics is a field that combines robotics and artificial intelligence...",
      "document_id": "chapter-2",
      "similarity_score": 0.78,
      "metadata": {
        "page_number": 45,
        "url": "https://book.example.com/chapter-2#page-45"
      }
    }
  ],
  "has_sources": true,
  "confidence": 0.85,
  "timestamp": "2025-12-22T10:30:00Z"
}
```

## Troubleshooting

### Common Issues
- **400 errors**: Check that your query meets the length requirements (1-2000 characters)
- **500 errors**: Verify that your API keys are correct and that the Qdrant collection exists
- **No results**: Ensure that the vector database has been populated with book content

### Environment Variables
Make sure all required environment variables are set correctly:
- `OPENAI_API_KEY`: Should start with "sk-"
- `QDRANT_URL`: Should be a valid HTTPS URL
- `QDRANT_API_KEY`: Should be a valid API key for your Qdrant instance
- `COLLECTION_NAME`: Should match the name of your vector collection