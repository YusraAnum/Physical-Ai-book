# Retrieval API Documentation

This document provides comprehensive documentation for the RAG retrieval API endpoints.

## Base URL

All API endpoints are prefixed with `/retrieval` and follow the format: `http://localhost:8000/retrieval/{endpoint}`

## Endpoints

### POST /retrieval/query

Submit a text query and retrieve semantically similar content chunks.

#### Request

**Content-Type:** `application/json`

**Body Parameters:**
- `query` (string, required): The text query to search for
- `top_k` (integer, optional): Number of top results to return (default: 5, min: 1, max: 10)
- `min_score` (float, optional): Minimum similarity score threshold (default: 0.5, range: 0.0-1.0)

**Example Request:**
```json
{
  "query": "Explain ROS 2 middleware concepts",
  "top_k": 5,
  "min_score": 0.5
}
```

#### Response

**Success Response (200 OK):**
```json
{
  "query": "Explain ROS 2 middleware concepts",
  "results": [
    {
      "id": "chunk-id-1",
      "content": "Content of the retrieved chunk...",
      "score": 0.92,
      "metadata": {
        "url": "http://example.com/page",
        "chunk_id": "chunk-id-1",
        "source_title": "Page Title",
        "created_at": "2025-12-20T10:30:00Z",
        "additional_metadata": {}
      }
    }
  ],
  "query_time_ms": 123.45,
  "total_chunks_searched": 100
}
```

**Error Responses:**
- `400 Bad Request`: Invalid parameters (e.g., top_k out of range)
- `422 Unprocessable Entity`: Missing required fields or empty query
- `500 Internal Server Error`: Internal processing error
- `503 Service Unavailable`: External service (Qdrant/Cohere) unavailable

### GET /retrieval/health

Check the health status of the retrieval service.

#### Response

**Success Response (200 OK):**
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "cohere_connected": true,
  "last_heartbeat": "2025-12-20T10:30:00.123456"
}
```

**Error Response:**
- `500 Internal Server Error`: Health check failed

### POST /retrieval/validate

Validate that the retrieval pipeline is working correctly by running a test query against known content.

#### Request

**Content-Type:** `application/json`

**Body Parameters:**
- `test_query` (string, required): The query text to test
- `expected_urls` (array[string], required): List of URLs that should be retrieved
- `threshold` (float, optional): Minimum accuracy threshold (default: 0.9, range: 0.0-1.0)

**Example Request:**
```json
{
  "test_query": "Test query for validation",
  "expected_urls": ["http://example.com/page1", "http://example.com/page2"],
  "threshold": 0.9
}
```

#### Response

**Success Response (200 OK):**
```json
{
  "success": true,
  "accuracy": 0.95,
  "retrieved_urls": ["http://example.com/page1", "http://example.com/page2"],
  "matches_expected": true,
  "details": {
    "top_match_accuracy": 0.92,
    "content_similarity": 0.95,
    "metadata_correctness": 1.0
  }
}
```

**Error Responses:**
- `400 Bad Request`: Invalid parameters (e.g., threshold out of range)
- `422 Unprocessable Entity`: Missing required fields
- `500 Internal Server Error`: Validation failed

### POST /retrieval/validate-full

Validate the full retrieval pipeline including content accuracy and metadata validation.

#### Request

Same as `/retrieval/validate` endpoint.

#### Response

**Success Response (200 OK):**
```json
{
  "success": true,
  "accuracy": 0.95,
  "retrieved_urls": ["http://example.com/page1", "http://example.com/page2"],
  "matches_expected": true,
  "details": {
    "content_verification_score": 0.98,
    "metadata_validation_score": 1.0,
    "overall_pipeline_passes": true,
    "content_similarity": 0.95,
    "retrieved_count": 2,
    "expected_count": 2
  }
}
```

**Error Responses:**
- `400 Bad Request`: Invalid parameters
- `422 Unprocessable Entity`: Missing required fields
- `500 Internal Server Error`: Full pipeline validation failed

## Error Handling

The API follows standard HTTP status codes for error responses:

- `200 OK`: Request successful
- `400 Bad Request`: Client error with request parameters
- `422 Unprocessable Entity`: Request parameters are invalid
- `500 Internal Server Error`: Internal server error
- `503 Service Unavailable`: External service unavailable

## Performance

The retrieval pipeline is designed to complete queries within 2 seconds for optimal user experience.