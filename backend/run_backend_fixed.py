import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# Set correct environment variables for consistent embedding
os.environ.setdefault('OPENAI_API_KEY', 'your_openai_api_key_here')  # Replace with your actual OpenAI key
os.environ.setdefault('QDRANT_URL', 'http://localhost:6333')
os.environ.setdefault('EMBEDDING_MODEL', 'openai')
os.environ.setdefault('EMBEDDING_DIMENSION', '1536')

# Import and run the main app
from main import app
import uvicorn

if __name__ == "__main__":
    print("Starting RAG backend service on http://localhost:8000")
    print("Configuration: embedding_model=openai, embedding_dimension=1536")
    uvicorn.run(app, host="0.0.0.0", port=8000)