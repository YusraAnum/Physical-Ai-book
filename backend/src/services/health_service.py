"""
Health service for checking the status of various system components.
"""
from typing import Dict, Any, List
import asyncio
import aiohttp
from ..services.qdrant_client_wrapper import QdrantClientWrapper
from ..services.cohere_client_wrapper import CohereClientWrapper


class HealthService:
    """
    Service class for performing health checks on system components.
    """
    def __init__(self):
        """
        Initialize the health service with required client wrappers.
        """
        self.qdrant_client = QdrantClientWrapper()
        self.cohere_client = CohereClientWrapper()

    def check_overall_health(self) -> Dict[str, Any]:
        """
        Check the health of all system components.

        Returns:
            Dictionary with overall health status and component details
        """
        qdrant_healthy = self.check_qdrant_health()
        cohere_healthy = self.check_cohere_health()

        all_healthy = qdrant_healthy and cohere_healthy

        return {
            "status": "healthy" if all_healthy else "unhealthy",
            "timestamp": self._get_current_timestamp(),
            "components": {
                "qdrant": {
                    "status": "healthy" if qdrant_healthy else "unhealthy",
                    "connected": qdrant_healthy
                },
                "cohere": {
                    "status": "healthy" if cohere_healthy else "unhealthy",
                    "connected": cohere_healthy
                }
            },
            "overall_healthy": all_healthy
        }

    def check_qdrant_health(self) -> bool:
        """
        Check the health of the Qdrant connection.

        Returns:
            True if Qdrant is healthy, False otherwise
        """
        try:
            return self.qdrant_client.check_health()
        except Exception:
            return False

    def check_cohere_health(self) -> bool:
        """
        Check the health of the Cohere API connection.

        Returns:
            True if Cohere is healthy, False otherwise
        """
        try:
            return self.cohere_client.check_health()
        except Exception:
            return False

    def get_detailed_health_info(self) -> Dict[str, Any]:
        """
        Get detailed health information for all components.

        Returns:
            Dictionary with detailed health information
        """
        qdrant_health = self._get_detailed_qdrant_health()
        cohere_health = self._get_detailed_cohere_health()

        return {
            "timestamp": self._get_current_timestamp(),
            "qdrant": qdrant_health,
            "cohere": cohere_health,
            "summary": {
                "total_components": 2,
                "healthy_components": sum([
                    qdrant_health.get("connected", False),
                    cohere_health.get("connected", False)
                ]),
                "unhealthy_components": sum([
                    not qdrant_health.get("connected", False),
                    not cohere_health.get("connected", False)
                ])
            }
        }

    def _get_detailed_qdrant_health(self) -> Dict[str, Any]:
        """
        Get detailed health information for Qdrant.

        Returns:
            Dictionary with detailed Qdrant health information
        """
        try:
            # Test basic connectivity
            connected = self.qdrant_client.check_health()

            # Get collection info if connected
            collection_info = {}
            if connected:
                collection_info = self.qdrant_client.get_collection_info()

            return {
                "connected": connected,
                "collection_info": collection_info,
                "last_checked": self._get_current_timestamp()
            }
        except Exception as e:
            return {
                "connected": False,
                "error": str(e),
                "last_checked": self._get_current_timestamp()
            }

    def _get_detailed_cohere_health(self) -> Dict[str, Any]:
        """
        Get detailed health information for Cohere.

        Returns:
            Dictionary with detailed Cohere health information
        """
        try:
            # Test basic connectivity
            connected = self.cohere_client.check_health()

            # Get model info if connected
            model_info = {}
            if connected:
                model_info = self.cohere_client.get_model_info()

            return {
                "connected": connected,
                "model_info": model_info,
                "last_checked": self._get_current_timestamp()
            }
        except Exception as e:
            return {
                "connected": False,
                "error": str(e),
                "last_checked": self._get_current_timestamp()
            }

    def _get_current_timestamp(self) -> str:
        """
        Get the current timestamp in ISO format.

        Returns:
            Current timestamp as ISO string
        """
        from datetime import datetime
        return datetime.now().isoformat()

    async def check_external_dependencies_async(self) -> Dict[str, Any]:
        """
        Check health of external dependencies asynchronously.

        Returns:
            Dictionary with external dependency health status
        """
        async def check_url(url: str) -> Dict[str, Any]:
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.get(url, timeout=5) as response:
                        return {
                            "url": url,
                            "status": "healthy",
                            "http_status": response.status,
                            "reachable": response.status == 200
                        }
            except Exception as e:
                return {
                    "url": url,
                    "status": "unhealthy",
                    "error": str(e),
                    "reachable": False
                }

        # Check external dependencies if any
        external_checks = []

        return {
            "checks": external_checks,
            "timestamp": self._get_current_timestamp()
        }