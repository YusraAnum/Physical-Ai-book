/**
 * Error handling utilities for the RAG chat interface
 */

/**
 * Standardized error response format
 */
export class ChatError extends Error {
  constructor(message, code, status = null) {
    super(message);
    this.name = 'ChatError';
    this.code = code;
    this.status = status;
  }
}

/**
 * Handle API errors and return user-friendly messages
 * @param {Error} error - The error object
 * @returns {Object} - Formatted error object with user-friendly message
 */
export const handleApiError = (error) => {
  let userMessage = 'An unexpected error occurred. Please try again.';
  let code = 'UNKNOWN_ERROR';

  if (error.name === 'TypeError' && error.message.includes('fetch')) {
    userMessage = 'Unable to connect to the server. Please check your connection.';
    code = 'NETWORK_ERROR';
  } else if (error.status) {
    switch (error.status) {
      case 400:
        userMessage = 'Invalid request. Please check your input.';
        code = 'BAD_REQUEST';
        break;
      case 422:
        userMessage = 'Invalid input format. Please check your query.';
        code = 'VALIDATION_ERROR';
        break;
      case 500:
        userMessage = 'Server error. The service is temporarily unavailable.';
        code = 'SERVER_ERROR';
        break;
      case 503:
        userMessage = 'Service unavailable. Please try again later.';
        code = 'SERVICE_UNAVAILABLE';
        break;
      default:
        userMessage = `Server error (${error.status}). Please try again.`;
        code = `HTTP_${error.status}`;
    }
  } else if (error.message) {
    if (error.message.includes('timeout')) {
      userMessage = 'Request timed out. Please try again.';
      code = 'TIMEOUT_ERROR';
    } else if (error.message.includes('network')) {
      userMessage = 'Network error. Please check your connection.';
      code = 'NETWORK_ERROR';
    }
  }

  return {
    message: userMessage,
    code,
    originalError: error
  };
};

/**
 * Validate query input
 * @param {string} query - The query string to validate
 * @returns {Array} - Array of validation errors
 */
export const validateQuery = (query) => {
  const errors = [];

  if (!query || typeof query !== 'string') {
    errors.push('Query is required and must be a string');
  } else if (query.trim().length === 0) {
    errors.push('Query cannot be empty');
  } else if (query.length > 1000) {
    errors.push('Query must be less than 1000 characters');
  }

  return errors;
};

/**
 * Validate context input
 * @param {string} context - The context string to validate
 * @returns {Array} - Array of validation errors
 */
export const validateContext = (context) => {
  const errors = [];

  if (context && typeof context !== 'string') {
    errors.push('Context must be a string if provided');
  } else if (context && context.length > 5000) {
    errors.push('Context must be less than 5000 characters');
  }

  return errors;
};

/**
 * Validate session ID
 * @param {string} sessionId - The session ID to validate
 * @returns {Array} - Array of validation errors
 */
export const validateSessionId = (sessionId) => {
  const errors = [];

  if (sessionId && typeof sessionId !== 'string') {
    errors.push('Session ID must be a string if provided');
  } else if (sessionId && !/^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i.test(sessionId)) {
    errors.push('Session ID must be a valid UUID if provided');
  }

  return errors;
};

/**
 * Create a standardized error response for the UI
 * @param {string} message - The error message
 * @param {string} type - The error type (e.g., 'network', 'validation', 'server')
 * @returns {Object} - Standardized error response object
 */
export const createErrorResponse = (message, type = 'general') => {
  return {
    success: false,
    error: {
      message,
      type,
      timestamp: new Date().toISOString()
    }
  };
};

/**
 * Create a standardized success response for the UI
 * @param {any} data - The response data
 * @returns {Object} - Standardized success response object
 */
export const createSuccessResponse = (data) => {
  return {
    success: true,
    data,
    timestamp: new Date().toISOString()
  };
};