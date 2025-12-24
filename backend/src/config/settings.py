from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    # OpenAI settings
    openai_api_key: str
    openai_model: str = "gpt-3.5-turbo"

    # Qdrant settings
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    collection_name: str = "book-content"

    # Application settings
    app_title: str = "RAG Agent Backend"
    app_description: str = "RAG-enabled backend agent for question answering over book content"
    app_version: str = "1.0.0"
    debug: bool = False

    class Config:
        env_file = ".env" if os.path.exists(".env") else None  # Only load .env if it exists
        case_sensitive = True
        extra = "forbid"  # Forbid extra fields to catch typos in env vars


settings = Settings()