/**
 * Session Management Service for RAG Chat
 * Handles conversation context and session persistence
 */

class SessionManager {
  constructor() {
    this.currentSessionId = null;
    this.conversationHistory = [];
    this.storageKey = 'rag-chat-session';
    this.loadSession();
  }

  /**
   * Generate a new session ID
   * @returns {string} A new UUID
   */
  generateSessionId() {
    // Simple UUID v4 generator
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
  }

  /**
   * Initialize a new session
   * @returns {string} The new session ID
   */
  initSession() {
    this.currentSessionId = this.generateSessionId();
    this.conversationHistory = [];
    this.saveSession();
    return this.currentSessionId;
  }

  /**
   * Get the current session ID
   * @returns {string|null} The current session ID or null if no session exists
   */
  getSessionId() {
    if (!this.currentSessionId) {
      this.currentSessionId = this.generateSessionId();
      this.saveSession();
    }
    return this.currentSessionId;
  }

  /**
   * Add a message to the conversation history
   * @param {Object} message - The message to add
   * @param {string} message.id - Message ID
   * @param {string} message.content - Message content
   * @param {string} message.sender - Sender ('USER' or 'ASSISTANT')
   * @param {string} message.timestamp - Timestamp
   */
  addMessage(message) {
    this.conversationHistory.push(message);
    this.saveSession();
  }

  /**
   * Get the conversation history
   * @returns {Array} Array of messages in the conversation
   */
  getConversationHistory() {
    return [...this.conversationHistory]; // Return a copy to prevent direct mutation
  }

  /**
   * Clear the current session
   */
  clearSession() {
    this.currentSessionId = null;
    this.conversationHistory = [];
    this.saveSession();
  }

  /**
   * Save the current session to localStorage
   */
  saveSession() {
    try {
      const sessionData = {
        sessionId: this.currentSessionId,
        conversationHistory: this.conversationHistory,
        lastActive: new Date().toISOString()
      };
      localStorage.setItem(this.storageKey, JSON.stringify(sessionData));
    } catch (error) {
      console.warn('Failed to save session to localStorage:', error);
    }
  }

  /**
   * Load session from localStorage
   */
  loadSession() {
    try {
      const sessionData = localStorage.getItem(this.storageKey);
      if (sessionData) {
        const parsed = JSON.parse(sessionData);
        this.currentSessionId = parsed.sessionId;
        this.conversationHistory = parsed.conversationHistory || [];
      } else {
        // Initialize a new session if none exists
        this.initSession();
      }
    } catch (error) {
      console.warn('Failed to load session from localStorage:', error);
      // Initialize a new session if loading fails
      this.initSession();
    }
  }

  /**
   * Get session data for API requests
   * @returns {Object} Session data including ID and context
   */
  getSessionData() {
    return {
      sessionId: this.getSessionId(),
      hasHistory: this.conversationHistory.length > 0
    };
  }
}

// Create a singleton instance
const sessionManager = new SessionManager();

export default sessionManager;