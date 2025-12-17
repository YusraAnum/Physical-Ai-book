import React, { useState, useEffect } from 'react';

// Theme context for Physical AI & Humanoid Robotics textbook
const ThemeContext = React.createContext();

// Default theme settings
const defaultThemeSettings = {
  id: 'physical-ai-theme',
  name: 'Dark Sci-Fi',
  colors: {
    primary: '#8B5CF6', // Neon purple
    secondary: '#38BDF8', // Electric blue
    accentGlow: '#22D3EE', // Soft cyan
    backgroundStart: '#0B0F1A',
    backgroundEnd: '#111827',
    textPrimary: '#E5E7EB',
    textMuted: '#9CA3AF',
  },
  typography: {
    headings: 'Orbitron, Poppins, Inter, system-ui, -apple-system, sans-serif',
    body: 'Inter, Poppins, system-ui, -apple-system, sans-serif',
    code: 'JetBrains Mono, Courier New, monospace',
  },
  layout: {
    maxWidth: '1200px',
    borderRadius: '0.5rem',
    spacing: {
      xs: '0.25rem',
      sm: '0.5rem',
      md: '1rem',
      lg: '1.5rem',
      xl: '2rem',
    },
  },
};

// Language context for multi-language support
const LanguageContext = React.createContext();

// Default language settings
const defaultLanguageSettings = {
  id: 'lang-context',
  currentLanguage: 'en',
  availableLanguages: [
    { code: 'en', name: 'EN', rtl: false },
    { code: 'ur', name: 'اردو', rtl: true },
  ],
  rtlEnabled: false,
};

// Custom Root component for the Physical AI textbook
export default function Root({ children }) {
  // Theme state
  const [theme, setTheme] = useState(defaultThemeSettings);

  // Initialize language state safely for server-side rendering
  const [language, setLanguage] = useState(() => {
    // Check if we're running in the browser
    if (typeof window !== 'undefined') {
      const savedLang = localStorage.getItem('preferred-language');
      return {
        ...defaultLanguageSettings,
        currentLanguage: savedLang || 'en',
        rtlEnabled: savedLang === 'ur',
      };
    }

    // Server-side default
    return defaultLanguageSettings;
  });

  // Effect to handle client-side operations only
  useEffect(() => {
    // Only run in browser environment
    if (typeof window === 'undefined') return;

    // Apply RTL to document when language changes
    const isRTLLanguage = language.availableLanguages.some(
      lang => lang.code === language.currentLanguage && lang.rtl
    );

    if (isRTLLanguage) {
      document.documentElement.setAttribute('dir', 'rtl');
      document.documentElement.classList.add('rtl-layout');
    } else {
      document.documentElement.setAttribute('dir', 'ltr');
      document.documentElement.classList.remove('rtl-layout');
    }

    // Update language context in local storage
    localStorage.setItem('preferred-language', language.currentLanguage);
  }, [language.currentLanguage]);

  // Function to switch language
  const switchLanguage = (langCode) => {
    const selectedLang = language.availableLanguages.find(lang => lang.code === langCode);
    if (selectedLang) {
      setLanguage(prev => ({
        ...prev,
        currentLanguage: langCode,
        rtlEnabled: selectedLang.rtl,
      }));
    }
  };

  // Theme provider value
  const themeValue = {
    theme,
    setTheme,
    // Add any theme-related functions here
    updateTheme: (newTheme) => setTheme(prev => ({ ...prev, ...newTheme })),
  };

  // Language provider value
  const languageValue = {
    language,
    switchLanguage,
  };

  return (
    <ThemeContext.Provider value={themeValue}>
      <LanguageContext.Provider value={languageValue}>
        {children}
      </LanguageContext.Provider>
    </ThemeContext.Provider>
  );
}

// Hook to use theme context
export const useTheme = () => {
  const context = React.useContext(ThemeContext);
  if (!context) {
    throw new Error('useTheme must be used within a ThemeProvider');
  }
  return context;
};

// Hook to use language context
export const useLanguage = () => {
  const context = React.useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};