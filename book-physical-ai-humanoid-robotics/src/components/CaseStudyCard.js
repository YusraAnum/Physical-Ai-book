import React from 'react';

const CaseStudyCard = ({ title, children, icon = '⚙️' }) => {
  return (
    <div className="card card--case-study">
      <div className="card__header">
        <h3>{icon} {title}</h3>
      </div>
      <div className="card__body">
        {children}
      </div>
    </div>
  );
};

export default CaseStudyCard;