import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# Set default environment variables for local development
os.environ.setdefault('OPENAI_API_KEY', 'dummy-key-for-testing')  # Replace with your actual OpenAI key
os.environ.setdefault('QDRANT_URL', 'http://localhost:6333')  # For local Qdrant
os.environ.setdefault('QDRANT_API_KEY', '')  # Empty for local instance

# Import and run the main app
from main import app
import uvicorn

if __name__ == "__main__":
    print("Starting RAG backend service on http://localhost:8000")
    print("Make sure to set your OPENAI_API_KEY environment variable")
    uvicorn.run(app, host="0.0.0.0", port=8000)