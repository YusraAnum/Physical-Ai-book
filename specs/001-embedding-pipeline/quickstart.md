# Quickstart Guide: Embedding Pipeline for Docusaurus Content

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant API key and cluster URL (or local instance)

## Setup

### 1. Create the Backend Directory

```bash
mkdir backend
cd backend
```

### 2. Initialize UV Package Manager

```bash
uv init
```

### 3. Install Dependencies

```bash
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv numpy
```

### 4. Set Up Environment Variables

Create a `.env` file in the backend directory:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Configuration

The pipeline can be configured via environment variables or by modifying the configuration in the main.py file:

- `BASE_URL`: The root URL of the Docusaurus site to crawl
- `CHUNK_SIZE`: Maximum size of text chunks in characters (default: 1000)
- `CHUNK_OVERLAP`: Number of characters to overlap between chunks (default: 200)
- `COHERE_MODEL`: Name of the Cohere embedding model (default: "embed-multilingual-v3.0")
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "rag-embeddings")

## Running the Pipeline

### 1. Prepare your main.py file

Create a `main.py` file with the complete implementation containing:
- `get_all_urls` function to discover all accessible URLs
- `extract_text_from_url` function to extract clean text content
- `chunk_text` function to split text into appropriately sized chunks
- `create_collection` function to initialize the Qdrant collection
- `save_chunk_to_qdrant` function to store embeddings with metadata
- Main execution function to orchestrate the pipeline

### 2. Execute the Pipeline

```bash
python main.py
```

## Expected Output

The pipeline will:
1. Crawl the specified Docusaurus site and discover all accessible URLs
2. Extract clean text content from each page
3. Split the content into semantic chunks
4. Generate embeddings using Cohere API
5. Store the embeddings in Qdrant with metadata
6. Provide progress updates during execution

## Example Usage

```python
# To run with custom parameters
import os
os.environ['BASE_URL'] = 'https://your-docusaurus-site.com'
os.environ['CHUNK_SIZE'] = '1500'
os.environ['CHUNK_OVERLAP'] = '300'

# Then run the main function
```

## Troubleshooting

### Common Issues

1. **Rate Limiting**: If you encounter rate limiting errors, increase the `RATE_LIMIT_DELAY` value
2. **Memory Issues**: For large sites, reduce the `BATCH_SIZE` to process fewer documents at once
3. **API Key Errors**: Verify that your Cohere and Qdrant API keys are correct and have sufficient quota
4. **Crawling Issues**: Check that the target site is accessible and doesn't have strict anti-bot measures

### Verification

To verify the pipeline worked correctly:
1. Check that the Qdrant collection was created with the expected name
2. Verify that embeddings were stored by querying similar vectors
3. Confirm that metadata includes the correct source URLs and document information