import React from 'react';
import { useLanguage } from '../theme/Root';

const LanguageToggle = () => {
  const { language, switchLanguage } = useLanguage();

  const handleLanguageChange = (langCode) => {
    switchLanguage(langCode);
  };

  return (
    <div className="language-toggle">
      <button
        onClick={() => handleLanguageChange('en')}
        className={`lang-button ${language.currentLanguage === 'en' ? 'active' : ''}`}
        aria-label="Switch to English"
      >
        EN
      </button>
      <span className="lang-separator"> | </span>
      <button
        onClick={() => handleLanguageChange('ur')}
        className={`lang-button ${language.currentLanguage === 'ur' ? 'active' : ''}`}
        aria-label="Switch to Urdu"
      >
        اردو
      </button>
    </div>
  );
};

export default LanguageToggle;