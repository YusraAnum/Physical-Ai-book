/**
 * Text Selection Utility for RAG Chat Integration
 * Provides functionality to capture selected text and integrate with the chat interface
 */

class TextSelectionUtility {
  constructor() {
    this.selectedText = null;
    this.selectionRect = null;
    this.onTextSelected = null;
    this.init();
  }

  init() {
    // Listen for text selection events
    document.addEventListener('mouseup', this.handleTextSelection.bind(this));
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        this.clearSelection();
      }
    });
  }

  handleTextSelection() {
    // Get the current text selection
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (selectedText) {
      // Get the bounding rectangle of the selection for positioning
      if (selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        this.selectionRect = range.getBoundingClientRect();
      }

      this.selectedText = selectedText;

      // If there's a callback for when text is selected, call it
      if (this.onTextSelected && typeof this.onTextSelected === 'function') {
        this.onTextSelected(selectedText, this.selectionRect);
      }
    } else {
      this.selectedText = null;
      this.selectionRect = null;
    }
  }

  /**
   * Get the currently selected text
   * @returns {string|null} The selected text or null if no text is selected
   */
  getSelectedText() {
    return this.selectedText;
  }

  /**
   * Clear the current selection
   */
  clearSelection() {
    this.selectedText = null;
    this.selectionRect = null;
    window.getSelection().removeAllRanges();
  }

  /**
   * Set a callback function to be called when text is selected
   * @param {Function} callback - Function to call when text is selected
   */
  setOnTextSelectedCallback(callback) {
    this.onTextSelected = callback;
  }

  /**
   * Create a context menu for selected text
   * @param {Function} onAskQuestion - Function to call when "Ask Question" is clicked
   * @returns {HTMLElement} The context menu element
   */
  createContextMenu(onAskQuestion) {
    // Remove any existing context menu
    const existingMenu = document.getElementById('rag-context-menu');
    if (existingMenu) {
      existingMenu.remove();
    }

    const contextMenu = document.createElement('div');
    contextMenu.id = 'rag-context-menu';
    contextMenu.className = 'rag-context-menu';
    contextMenu.style.position = 'absolute';
    contextMenu.style.display = 'none';
    contextMenu.style.backgroundColor = '#4a6cf7';
    contextMenu.style.color = 'white';
    contextMenu.style.borderRadius = '4px';
    contextMenu.style.padding = '4px 8px';
    contextMenu.style.zIndex = '10000';
    contextMenu.style.fontSize = '14px';
    contextMenu.style.cursor = 'pointer';
    contextMenu.style.boxShadow = '0 2px 10px rgba(0,0,0,0.2)';
    contextMenu.style.userSelect = 'none';

    contextMenu.textContent = 'Ask Question';
    contextMenu.addEventListener('click', () => {
      if (onAskQuestion && typeof onAskQuestion === 'function') {
        onAskQuestion(this.selectedText);
      }
      contextMenu.style.display = 'none';
    });

    document.body.appendChild(contextMenu);

    // Position the context menu near the selection
    if (this.selectionRect) {
      contextMenu.style.top = `${this.selectionRect.bottom + window.scrollY + 5}px`;
      contextMenu.style.left = `${this.selectionRect.left + window.scrollX}px`;
      contextMenu.style.display = 'block';
    }

    // Hide the context menu when clicking elsewhere
    const hideMenu = (e) => {
      if (!contextMenu.contains(e.target)) {
        contextMenu.style.display = 'none';
        document.removeEventListener('click', hideMenu);
      }
    };

    setTimeout(() => {
      document.addEventListener('click', hideMenu);
    }, 10);

    return contextMenu;
  }

  /**
   * Highlight selected text with a custom style
   */
  highlightSelection() {
    if (!this.selectedText) return;

    const selection = window.getSelection();
    if (selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const span = document.createElement('span');
      span.style.backgroundColor = 'rgba(74, 108, 247, 0.2)';
      span.style.borderBottom = '2px solid #4a6cf7';
      span.style.borderRadius = '2px';

      range.surroundContents(span);
    }
  }

  /**
   * Remove highlighting from selected text
   */
  removeHighlight() {
    const highlightedSpans = document.querySelectorAll('span[style*="rgba(74, 108, 247, 0.2)"]');
    highlightedSpans.forEach(span => {
      const parent = span.parentNode;
      while (span.firstChild) {
        parent.insertBefore(span.firstChild, span);
      }
      parent.removeChild(span);
    });
  }
}

// Initialize the text selection utility
const textSelectionUtility = new TextSelectionUtility();

// Make it available globally
window.RAGTextSelection = textSelectionUtility;

// Export for module usage if needed
if (typeof module !== 'undefined' && module.exports) {
  module.exports = TextSelectionUtility;
}

/**
 * CSS for the context menu (to be added to the page)
 */
const contextMenuCSS = `
  .rag-context-menu {
    position: absolute;
    background-color: #4a6cf7;
    color: white;
    border-radius: 4px;
    padding: 4px 8px;
    z-index: 10000;
    font-size: 14px;
    cursor: pointer;
    box-shadow: 0 2px 10px rgba(0,0,0,0.2);
    user-select: none;
    white-space: nowrap;
  }

  .rag-context-menu:hover {
    background-color: #3a5ce5;
  }
`;

// Add the CSS to the page
const style = document.createElement('style');
style.textContent = contextMenuCSS;
document.head.appendChild(style);

// Remove the export default statement as this file is loaded as a script in the browser
// The utility is already available globally as window.RAGTextSelection