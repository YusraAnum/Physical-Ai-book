import React, { useState, useEffect } from 'react';
import ChatWindow from '../ChatInterface/ChatWindow';
import '../ChatInterface/ChatWindow.css';

/**
 * Floating Chat Widget Component
 * A persistent chat widget that appears on all book pages
 */
const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMounted, setIsMounted] = useState(false);

  // Initialize after component mounts to ensure DOM is ready
  useEffect(() => {
    setIsMounted(true);

    // Check for existing text selection functionality
    if (typeof window !== 'undefined' && window.RAGTextSelection) {
      // Set up callback for when text is selected
      window.RAGTextSelection.setOnTextSelectedCallback((selectedText, rect) => {
        // Show context menu when text is selected
        window.RAGTextSelection.createContextMenu((text) => {
          // Open chat and set context
          setIsOpen(true);
        });
      });
    }
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  // Don't render until mounted to avoid SSR issues
  if (!isMounted) {
    return null;
  }

  return (
    <>
      {!isOpen && (
        <button
          className="chat-toggle-button"
          onClick={toggleChat}
          aria-label="Open chat"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#4a6cf7',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            zIndex: 999,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
        >
          💬
        </button>
      )}

      {isOpen && (
        <ChatWindow
          isOpen={isOpen}
          onClose={closeChat}
          onSubmitQuery={async (query, context = null) => {
            // Import the submitQuery function
            const { submitQuery } = await import('../../services/rag-api');
            return submitQuery({ query, context });
          }}
        />
      )}
    </>
  );
};

export default FloatingChatWidget;