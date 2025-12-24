from fastapi import FastAPI
from ..config.settings import settings


def create_app() -> FastAPI:
    """Create and configure the FastAPI application"""
    app = FastAPI(
        title=settings.app_title,
        description=settings.app_description,
        version=settings.app_version,
        debug=settings.debug
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