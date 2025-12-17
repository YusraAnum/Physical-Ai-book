import React from 'react';
import OriginalFooter from '@theme-original/Footer';
import LanguageToggle from '../../components/LanguageToggle';

// Custom Footer component for Physical AI & Humanoid Robotics textbook
export default function Footer(props) {
  return (
    <>
      <footer className="footer">
        <div className="container">
          <div className="footer__inner">
            <div className="footer__left">
              <span className="footer__icon">📘</span>
              <span>Physical AI & Humanoid Robotics</span>
            </div>

            <div className="footer__center">
              <span>Module | Chapter | Page number</span>
            </div>

            <div className="footer__right">
              <a href="#" className="footer__link" aria-label="GitHub">
                <span>🐙</span>
              </a>
              <a href="#" className="footer__link" aria-label="Documentation">
                <span>📚</span>
              </a>
              <a href="#" className="footer__link" aria-label="Feedback">
                <span>💬</span>
              </a>
              <LanguageToggle />
            </div>
          </div>
        </div>
      </footer>

      <OriginalFooter {...props} />
    </>
  );
}