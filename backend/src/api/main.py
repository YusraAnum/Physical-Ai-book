from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ..config.settings import settings


def create_app() -> FastAPI:
    """Create and configure the FastAPI application"""
    app = FastAPI(
        title=settings.app_title,
        description=settings.app_description,
        version=settings.app_version,
        debug=settings.debug
    )

    # Add CORS middleware to allow frontend communication
    # Use environment variable for flexible origin configuration
    frontend_origin = getattr(settings, 'cors_origin', 'http://localhost:3000')
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[frontend_origin, "http://localhost:3000", "http://127.0.0.1:3000", "*"],  # Allow multiple origins including all for dev
        allow_credentials=True,
        allow_methods=["*"],  # Allow all methods (GET, POST, etc.)
        allow_headers=["*"],  # Allow all headers
        # Expose headers that frontend might need to access
        expose_headers=["Access-Control-Allow-Origin", "Content-Type"]
    )

    # Include routes
    from .routes import chat, rag
    app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
    app.include_router(rag.router, prefix="/api", tags=["rag"])

    @app.get("/health")
    def health_check():
        return {"status": "healthy", "service": "rag-agent-backend"}

    return app


# Create the main app instance
app = create_app()