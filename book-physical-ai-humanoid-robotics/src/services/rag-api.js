/**
 * RAG API Service
 * Service for communicating with the RAG backend
 */

// Base URL for the RAG backend - configurable via window object for browser environments
// Can be set from HTML template or via environment variables during build
const RAG_API_BASE_URL =
  (typeof window !== 'undefined' && window.RAG_API_URL) ||
  (typeof process !== 'undefined' && process.env && (process.env.REACT_APP_API_URL || process.env.RAG_API_URL)) ||
  'http://localhost:8000';

/**
 * Submit a query to the RAG backend
 * @param {Object} queryData - The query data to send
 * @param {string} queryData.query - The user's question
 * @param {string} [queryData.context] - Optional context from selected text
 * @param {string} [queryData.session_id] - Optional session identifier
 * @returns {Promise<Object>} The response from the RAG backend
 */
const submitQuery = async (queryData) => {
  try {
    const response = await fetch(`${RAG_API_BASE_URL}/api/v1/rag/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(queryData),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error submitting query to RAG backend:', error);
    // Return a structured error response
    return {
      response: "",
      sources: [],
      status: "ERROR",
      session_id: queryData.session_id || null,
      error_message: error.message || "Network error occurred"
    };
  }
};

/**
 * Check the health of the RAG service
 * @returns {Promise<Object>} The health status response
 */
const checkHealth = async () => {
  try {
    const response = await fetch(`${RAG_API_BASE_URL}/api/v1/rag/health`);

    if (!response.ok) {
      throw new Error(`Health check failed with status: ${response.status}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error checking RAG service health:', error);
    throw error;
  }
};

/**
 * Validate query parameters
 * @param {Object} queryData - The query data to validate
 * @returns {Array} Array of validation errors
 */
const validateQuery = (queryData) => {
  const errors = [];

  if (!queryData.query || typeof queryData.query !== 'string') {
    errors.push('Query is required and must be a string');
  } else if (queryData.query.length < 1 || queryData.query.length > 1000) {
    errors.push('Query must be between 1 and 1000 characters');
  }

  if (queryData.context && typeof queryData.context === 'string' &&
      (queryData.context.length < 1 || queryData.context.length > 5000)) {
    errors.push('Context must be between 1 and 5000 characters if provided');
  }

  if (queryData.session_id && typeof queryData.session_id === 'string') {
    // Basic UUID validation (simplified)
    const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;
    if (!uuidRegex.test(queryData.session_id)) {
      errors.push('Session ID must be a valid UUID if provided');
    }
  }

  return errors;
};

// Create an object with all the functions
const RAGAPI = {
  submitQuery,
  checkHealth,
  validateQuery,
  RAG_API_BASE_URL
};

// Export for ES6 modules - both named and default exports
export default RAGAPI;
export { submitQuery, checkHealth, validateQuery, RAG_API_BASE_URL };