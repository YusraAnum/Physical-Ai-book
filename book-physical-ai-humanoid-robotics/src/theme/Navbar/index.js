import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import LanguageToggle from '../../components/LanguageToggle';

// Custom Navbar component for Physical AI & Humanoid Robotics textbook
export default function Navbar(props) {
  return (
    <>
      <nav className="navbar navbar--fixed-top">
        <div className="navbar__inner">
          <div className="navbar__items">
            <LanguageToggle />
          </div>
          <div className="navbar__items navbar__items--right">
            {/* Add any additional navbar items here */}
          </div>
        </div>
      </nav>

      <OriginalNavbar {...props} />
    </>
  );
}