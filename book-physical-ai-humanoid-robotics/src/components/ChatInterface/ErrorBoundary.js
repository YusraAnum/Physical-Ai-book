import React from 'react';

/**
 * Error Boundary Component for Chat Interface
 * Catches JavaScript errors anywhere in the child component tree
 * and displays a fallback UI instead of crashing the component.
 */
class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null, errorInfo: null };
  }

  static getDerivedStateFromError(error) {
    // Update state so the next render will show the fallback UI
    return { hasError: true };
  }

  componentDidCatch(error, errorInfo) {
    // You can also log the error to an error reporting service
    console.error('Chat Interface Error:', error, errorInfo);
    this.setState({
      error: error,
      errorInfo: errorInfo
    });
  }

  render() {
    if (this.state.hasError) {
      // You can render any custom fallback UI
      return (
        <div className="chat-error-boundary">
          <div className="error-content">
            <h3>Something went wrong with the chat interface</h3>
            <p>We're sorry, but the chat interface encountered an error.</p>
            <button
              onClick={() => this.setState({ hasError: false, error: null, errorInfo: null })}
              className="retry-button"
            >
              Try Again
            </button>
            {process.env.NODE_ENV === 'development' && (
              <details style={{ whiteSpace: 'pre-wrap' }}>
                <summary>Error details</summary>
                {this.state.error && this.state.error.toString()}
                <br />
                {this.state.errorInfo.componentStack}
              </details>
            )}
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;