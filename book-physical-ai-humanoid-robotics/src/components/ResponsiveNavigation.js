import React, { useState } from 'react';

const ResponsiveNavigation = () => {
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);

  const toggleMobileMenu = () => {
    setIsMobileMenuOpen(!isMobileMenuOpen);
  };

  return (
    <nav className="responsive-nav">
      <div className="nav-container">
        <div className="nav-brand">
          <a href="/" className="nav-logo">🤖 Physical AI</a>
        </div>

        <button
          className="mobile-menu-toggle"
          onClick={toggleMobileMenu}
          aria-label={isMobileMenuOpen ? "Close menu" : "Open menu"}
        >
          <span></span>
          <span></span>
          <span></span>
        </button>

        <div className={`nav-menu ${isMobileMenuOpen ? 'nav-menu--open' : ''}`}>
          <ul className="nav-list">
            <li className="nav-item">
              <a href="/modules" className="nav-link">Modules</a>
            </li>
            <li className="nav-item">
              <a href="/chapters" className="nav-link">Chapters</a>
            </li>
            <li className="nav-item">
              <a href="/resources" className="nav-link">Resources</a>
            </li>
            <li className="nav-item">
              <a href="/about" className="nav-link">About</a>
            </li>
          </ul>
        </div>
      </div>
    </nav>
  );
};

export default ResponsiveNavigation;