"""
Main FastAPI application for the RAG retrieval service.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Create FastAPI app
app = FastAPI(
    title="RAG Retrieval API",
    description="API for retrieving semantically similar content from vector database",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the RAG routers
from .api.routes.chat import router as chat_router
from .api.routes.rag import router as rag_router
app.include_router(chat_router, prefix="/api")
app.include_router(rag_router, prefix="/api")

# Root endpoint
@app.get("/")
async def root():
    return {"message": "RAG Retrieval API is running"}

# Health check for the overall app
@app.get("/health")
async def app_health():
    return {"status": "healthy", "service": "retrieval-api"}