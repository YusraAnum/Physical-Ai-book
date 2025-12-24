import React from 'react';

/**
 * Message Component
 * Displays individual chat messages with proper styling
 */
const Message = ({ message, isLoading = false }) => {
  if (isLoading) {
    return (
      <div className="message assistant">
        <div className="message-content">
          <div className="typing-indicator">
            <span></span>
            <span></span>
            <span></span>
          </div>
        </div>
      </div>
    );
  }

  // Handle error status
  if (message.status === 'ERROR') {
    return (
      <div className="message assistant">
        <div className="message-content">
          <div className="error-indicator">
            <p>⚠️ {message.error_message || 'There was an issue processing your request. Please try again.'}</p>
          </div>
        </div>
        {message.timestamp && (
          <small className="timestamp">
            {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
          </small>
        )}
      </div>
    );
  }

  // Handle empty status
  if (message.status === 'EMPTY') {
    return (
      <div className="message assistant">
        <div className="message-content">
          <p>{message.content || 'No relevant content found to answer your query.'}</p>
          <div className="empty-indicator">
            <p>🔍 No relevant content found in the book for your query.</p>
          </div>
        </div>
        {message.timestamp && (
          <small className="timestamp">
            {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
          </small>
        )}
      </div>
    );
  }

  return (
    <div className={`message ${message.sender.toLowerCase()}`}>
      <div className="message-content">
        <p>{message.content}</p>

        {message.sources && message.sources.length > 0 && (
          <div className="sources">
            <details>
              <summary>Sources</summary>
              <ul>
                {message.sources.map((source, index) => (
                  <li key={index}>
                    <strong>{source.title}</strong>: {source.content.substring(0, 100)}...
                    {source.page_reference && (
                      <small> - {source.page_reference}</small>
                    )}
                  </li>
                ))}
              </ul>
            </details>
          </div>
        )}
      </div>

      {message.timestamp && (
        <small className="timestamp">
          {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </small>
      )}
    </div>
  );
};

export default Message;