import React from 'react';

const Hero = () => {
  return (
    <section className="hero hero--primary padding-top--xl padding-bottom--lg">
      <div className="container">
        <div className="row">
          {/* Left side: Text content */}
          <div className="col col--6 fade-in">
            <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
            <p className="hero__subtitle">
              Master the future of AI-powered robotics with hands-on learning
            </p>
            <div className="hero__actions margin-top--lg">
              <button className="button button--primary button--lg margin-right--md pulse-cta">
                Start Reading
              </button>
              <button className="button button--secondary button--lg">
                View Sample
              </button>
            </div>
          </div>

          {/* Right side: Robot image with holographic tablet */}
          <div className="col col--6 text--center">
            <div className="hero__image-container position-relative">
              <img
                src="/img/heroimg.webp"
                alt="Humanoid robot holding a holographic tablet"
                className="hero__image robot-float"
                style={{ maxWidth: '100%', height: 'auto' }}
              />
              <div className="holographic-overlay position-absolute"
                   style={{
                     top: '20%',
                     left: '65%',
                     width: '80px',
                     height: '80px',
                     background: 'radial-gradient(circle, rgba(56, 189, 248, 0.3) 0%, transparent 70%)',
                     borderRadius: '50%',
                     filter: 'blur(10px)',
                     zIndex: '1'
                   }}></div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default Hero;