import React from 'react';

const DefinitionCard = ({ title, children, icon = '🤖' }) => {
  return (
    <div className="card card--definition">
      <div className="card__header">
        <h3>{icon} {title}</h3>
      </div>
      <div className="card__body">
        {children}
      </div>
    </div>
  );
};

export default DefinitionCard;