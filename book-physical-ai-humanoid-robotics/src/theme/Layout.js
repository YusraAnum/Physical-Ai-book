import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useTheme, useLanguage } from './Root';

// Custom Layout component for Physical AI & Humanoid Robotics textbook
export default function Layout(props) {
  const { theme } = useTheme();
  const { language } = useLanguage();

  // Apply theme-specific classes to body
  React.useEffect(() => {
    // Add theme classes to body for CSS targeting
    document.body.classList.add('physical-ai-theme');

    // Clean up on unmount
    return () => {
      document.body.classList.remove('physical-ai-theme');
    };
  }, []);

  return (
    <>
      {/* Theme-specific meta tags */}
      <meta name="theme-color" content={theme.colors.primary} />

      {/* Language-specific attributes */}
      <meta name="language" content={language.currentLanguage} />

      {/* Render the original layout */}
      <OriginalLayout {...props} />
    </>
  );
}