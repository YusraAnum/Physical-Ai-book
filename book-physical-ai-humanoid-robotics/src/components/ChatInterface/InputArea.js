import React, { useState, useEffect } from 'react';

/**
 * InputArea Component
 * Handles user input for the chat interface
 */
const InputArea = ({ onSubmit, isLoading, context = null, onContextChange = null }) => {
  const [inputValue, setInputValue] = useState('');
  const [localContext, setLocalContext] = useState(context);

  // Update local context when parent context changes
  useEffect(() => {
    setLocalContext(context);
  }, [context]);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSubmit(inputValue.trim(), localContext);
      setInputValue('');
      if (onContextChange) {
        onContextChange(null); // Clear context after submission
      } else {
        setLocalContext(null);
      }
    }
  };

  const handleClearContext = () => {
    if (onContextChange) {
      onContextChange(null);
    } else {
      setLocalContext(null);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="input-form">
      {localContext && (
        <div className="context-preview">
          <small>Context: "{localContext.substring(0, 50)}{localContext.length > 50 ? '...' : ''}"</small>
          <button
            type="button"
            onClick={handleClearContext}
            className="remove-context"
            aria-label="Remove context"
          >
            ×
          </button>
        </div>
      )}
      <div className="input-container">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder={localContext
            ? "Ask about the selected text..."
            : "Ask a question about the book content..."}
          disabled={isLoading}
          aria-label="Type your question"
        />
        <button
          type="submit"
          disabled={!inputValue.trim() || isLoading}
          aria-label="Send message"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </form>
  );
};

export default InputArea;