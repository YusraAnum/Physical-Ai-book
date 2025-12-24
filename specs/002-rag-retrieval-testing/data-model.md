# Data Model: RAG Retrieval Testing

## Overview
This document defines the data models for the RAG retrieval testing functionality, based on the entities identified in the feature specification.

## Core Entities

### Query
**Description**: A text input that needs to be semantically matched against stored content

**Fields**:
- `text`: string - The input query text to be processed
- `embedding`: list[float] - Vector representation of the query text
- `created_at`: datetime - Timestamp when the query was created

**Validation**:
- Text must not be empty
- Text must be within reasonable length limits (e.g., 1000 characters)

### Text Chunk
**Description**: A segment of the original document that has been processed and stored as vector embeddings

**Fields**:
- `id`: string - Unique identifier for the chunk
- `content`: string - The actual text content of the chunk
- `original_url`: string - URL of the source document
- `vector`: list[float] - Vector embedding of the text content
- `metadata`: dict - Additional metadata associated with the chunk

**Validation**:
- Content must not be empty
- Vector must have consistent dimensions
- ID must be unique within the system

### Metadata
**Description**: Information that identifies the source document (URL) and specific chunk (chunk_id) for traceability

**Fields**:
- `url`: string - URL of the source document
- `chunk_id`: string - Unique identifier for the specific chunk
- `source_title`: string - Title of the source document (optional)
- `created_at`: datetime - Timestamp when the metadata was created

**Validation**:
- URL must be a valid URL format
- Chunk ID must be unique and not empty

### Vector Embedding
**Description**: Numerical representation of text content that enables semantic similarity comparison

**Fields**:
- `vector`: list[float] - The actual embedding vector
- `model`: string - Name of the model used to generate the embedding
- `dimension`: int - Number of dimensions in the vector
- `text_hash`: string - Hash of the original text for integrity verification

**Validation**:
- Vector must have consistent dimensions across the system
- Model name must be valid and supported
- Dimension must match system expectations

### Retrieval Result
**Description**: The combination of matched text chunks and their associated metadata returned to the user

**Fields**:
- `query`: Query - The original query that generated this result
- `matches`: list[Text Chunk] - Top-K most similar text chunks
- `scores`: list[float] - Similarity scores for each match
- `metadata_list`: list[Metadata] - Metadata for each matched chunk
- `retrieval_time`: float - Time taken to retrieve the results
- `total_chunks_searched`: int - Total number of chunks searched

**Validation**:
- Matches and scores must have the same length
- Scores must be in descending order (most relevant first)
- Must contain at least one match unless no relevant results found

## Relationships

### Query → Retrieval Result
- One Query generates one Retrieval Result
- Relationship: 1:1 (for a single query execution)

### Retrieval Result → Text Chunk
- One Retrieval Result contains multiple Text Chunks
- Relationship: 1:Many (top-K matches)

### Text Chunk → Metadata
- One Text Chunk has one set of Metadata
- Relationship: 1:1 (each chunk has its own metadata)

## Data Flow

1. **Query Input**: User provides text query
2. **Embedding Generation**: Query text converted to vector embedding
3. **Similarity Search**: Vector compared against stored embeddings in Qdrant
4. **Result Assembly**: Top-K matches collected with scores and metadata
5. **Output Generation**: Retrieval Result created with all components

## Validation Rules

1. **Integrity Verification**: Retrieved content must match original source content with 99% accuracy
2. **Metadata Completeness**: All retrieval results must include complete metadata (URL, chunk_id)
3. **Performance Requirements**: End-to-end retrieval must complete within 2 seconds for 95% of queries
4. **Top-K Accuracy**: Query requests should return top-K matches with 90% semantic relevance accuracy