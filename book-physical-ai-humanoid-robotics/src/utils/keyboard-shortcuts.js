/**
 * Keyboard Shortcuts Utility for RAG Chat Interface
 * Handles keyboard shortcuts for chat interface access
 */

class KeyboardShortcuts {
  constructor() {
    this.shortcuts = new Map();
    this.enabled = true;
    this.init();
  }

  init() {
    // Add event listener for keyboard shortcuts
    document.addEventListener('keydown', this.handleKeyDown.bind(this));
  }

  /**
   * Register a keyboard shortcut
   * @param {string} key - The key combination (e.g., 'Ctrl+K', 'Cmd+K', 'Escape')
   * @param {Function} callback - The function to call when the shortcut is pressed
   * @param {string} description - Description of the shortcut
   */
  registerShortcut(key, callback, description) {
    this.shortcuts.set(key.toLowerCase(), { callback, description });
  }

  /**
   * Unregister a keyboard shortcut
   * @param {string} key - The key combination to remove
   */
  unregisterShortcut(key) {
    this.shortcuts.delete(key.toLowerCase());
  }

  /**
   * Handle keyboard events
   * @param {KeyboardEvent} event - The keyboard event
   */
  handleKeyDown(event) {
    if (!this.enabled) return;

    // Build the key combination string
    let keyCombo = '';
    if (event.ctrlKey) keyCombo += 'ctrl+';
    if (event.metaKey) keyCombo += 'cmd+'; // For Mac Command key
    if (event.shiftKey) keyCombo += 'shift+';
    if (event.altKey) keyCombo += 'alt+';

    // Add the actual key
    keyCombo += event.key.toLowerCase();

    // Check if this key combination is registered
    const shortcut = this.shortcuts.get(keyCombo);
    if (shortcut) {
      event.preventDefault();
      shortcut.callback(event);
    }

    // Special case: 'Escape' key (without modifiers)
    if (event.key === 'Escape' && !event.ctrlKey && !event.metaKey && !event.shiftKey && !event.altKey) {
      const escapeShortcut = this.shortcuts.get('escape');
      if (escapeShortcut) {
        event.preventDefault();
        escapeShortcut.callback(event);
      }
    }
  }

  /**
   * Enable keyboard shortcuts
   */
  enable() {
    this.enabled = true;
  }

  /**
   * Disable keyboard shortcuts
   */
  disable() {
    this.enabled = false;
  }

  /**
   * Get all registered shortcuts
   * @returns {Map} Map of all registered shortcuts
   */
  getShortcuts() {
    return new Map(this.shortcuts);
  }
}

// Create a singleton instance
const keyboardShortcuts = new KeyboardShortcuts();

// Register default shortcuts for the RAG chat
keyboardShortcuts.registerShortcut('ctrl+k', () => {
  // Toggle chat interface
  if (window && window.dispatchEvent) {
    window.dispatchEvent(new CustomEvent('ragChatToggle'));
  }
}, 'Toggle RAG Chat Interface');

keyboardShortcuts.registerShortcut('cmd+k', () => {
  // Toggle chat interface (Mac)
  if (window && window.dispatchEvent) {
    window.dispatchEvent(new CustomEvent('ragChatToggle'));
  }
}, 'Toggle RAG Chat Interface');

keyboardShortcuts.registerShortcut('escape', () => {
  // Close chat interface if open
  if (window && window.dispatchEvent) {
    window.dispatchEvent(new CustomEvent('ragChatClose'));
  }
}, 'Close RAG Chat Interface');

// Export for use in components
export default keyboardShortcuts;