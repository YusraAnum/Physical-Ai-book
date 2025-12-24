import React, { useState, useRef, useEffect } from 'react';
import Message from './Message';
import InputArea from './InputArea';
import './ChatWindow.css';

/**
 * ChatWindow Component
 * Main container for the chat interface
 */
const ChatWindow = ({ isOpen, onClose, onSubmitQuery }) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const handleSubmit = async (query, context = null) => {
    if (!query.trim()) return;

    try {
      setIsLoading(true);
      setError(null);

      // Add user message to UI immediately
      const userMessage = {
        id: Date.now(),
        content: query,
        sender: 'USER',
        timestamp: new Date().toISOString(),
        context: context
      };

      setMessages(prev => [...prev, userMessage]);

      // Submit query to backend
      const response = await onSubmitQuery(query, context);

      // Add assistant response to UI based on response status
      const assistantMessage = {
        id: Date.now() + 1,
        content: response.response,
        sender: 'ASSISTANT',
        timestamp: new Date().toISOString(),
        sources: response.sources || [],
        status: response.status,
        error_message: response.error_message
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      // Handle error case
      const errorMessage = {
        id: Date.now() + 1,
        content: "",
        sender: 'ASSISTANT',
        timestamp: new Date().toISOString(),
        sources: [],
        status: 'ERROR',
        error_message: err.message || 'An error occurred while processing your query.'
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="chat-window">
      <div className="chat-header">
        <h3>Book Assistant</h3>
        <button className="close-button" onClick={onClose} aria-label="Close chat">
          ×
        </button>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Ask me anything about the book content!</p>
            <p>I can help answer questions based on the book's text.</p>
          </div>
        ) : (
          messages.map((message) => (
            <Message key={message.id} message={message} />
          ))
        )}
        {isLoading && <Message isLoading={true} />}
        <div ref={messagesEndRef} />
      </div>

      {error && (
        <div className="error-message">
          Error: {error}
        </div>
      )}

      <div className="chat-input-area">
        <InputArea onSubmit={handleSubmit} isLoading={isLoading} />
      </div>
    </div>
  );
};

export default ChatWindow;