import React from 'react';

const ChapterLayoutWrapper = ({ children, title }) => {
  return (
    <div className="chapter-wrapper">
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <header className="chapter-header">
              <h1 className="chapter-title">{title}</h1>
            </header>

            <div className="chapter-content">
              {children}
            </div>

            <footer className="chapter-footer">
              <div className="chapter-navigation">
                <button className="button button--outline button--secondary">Previous</button>
                <button className="button button--primary">Next</button>
              </div>
            </footer>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChapterLayoutWrapper;