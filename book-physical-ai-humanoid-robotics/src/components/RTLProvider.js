import React, { useEffect, createContext, useContext } from 'react';

const RTLContext = createContext();

export const useRTL = () => {
  const context = useContext(RTLContext);
  if (!context) {
    throw new Error('useRTL must be used within an RTLProvider');
  }
  return context;
};

export const RTLProvider = ({ children, isRTL = false }) => {
  useEffect(() => {
    // Set the direction attribute on the html element
    document.documentElement.dir = isRTL ? 'rtl' : 'ltr';

    // Add RTL class for CSS targeting
    if (isRTL) {
      document.documentElement.classList.add('rtl');
    } else {
      document.documentElement.classList.remove('rtl');
    }
  }, [isRTL]);

  return (
    <RTLContext.Provider value={{ isRTL }}>
      {children}
    </RTLContext.Provider>
  );
};