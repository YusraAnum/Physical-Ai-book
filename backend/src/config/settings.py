from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    # OpenAI settings
    openai_api_key: str
    openai_model: str = "gpt-3.5-turbo"

    # Cohere settings (for embeddings)
    cohere_api_key: Optional[str] = None

    # Qdrant settings
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    collection_name: str = "book-content"

    # Application settings
    app_title: str = "RAG Agent Backend"
    app_description: str = "RAG-enabled backend agent for question answering over book content"
    app_version: str = "1.0.0"
    debug: bool = False

    # Embedding settings
    embedding_model: str = "openai"  # "openai" or "cohere"
    embedding_dimension: int = 1536  # 1536 for OpenAI ada-002, 1024 for Cohere

    class Config:
        env_file = ".env" if os.path.exists(".env") else None  # Only load .env if it exists
        case_sensitive = True
        extra = "forbid"  # Forbid extra fields to catch typos in env vars


settings = Settings()