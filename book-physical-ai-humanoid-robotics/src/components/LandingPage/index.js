import React from 'react';
import Hero from './Hero';

const LandingPage = () => {
  return (
    <div className="landing-page">
      <Hero />

      {/* Features Section */}
      <section className="features-section padding-top--lg padding-bottom--lg">
        <div className="container">
          <div className="row">
            <div className="col col--4">
              <div className="feature-card text--center padding--lg">
                <h3>🤖 Robotics</h3>
                <p>Learn about advanced robotics concepts and applications</p>
              </div>
            </div>
            <div className="col col--4">
              <div className="feature-card text--center padding--lg">
                <h3>🧠 AI Concepts</h3>
                <p>Understand artificial intelligence in robotics</p>
              </div>
            </div>
            <div className="col col--4">
              <div className="feature-card text--center padding--lg">
                <h3>⚙️ Hardware</h3>
                <p>Explore the hardware components of humanoid robots</p>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* Chapters Overview Section */}
      <section className="chapters-section padding-top--lg padding-bottom--lg">
        <div className="container">
          <div className="text--center margin-bottom--lg">
            <h2>What You'll Learn</h2>
            <p className="hero__subtitle">Comprehensive coverage of humanoid robotics concepts</p>
          </div>
          <div className="row">
            <div className="col col--3">
              <div className="card padding--md">
                <h4>ROS Nervous System</h4>
                <p>Understanding ROS 2 as the middleware for robot communication and coordination</p>
              </div>
            </div>
            <div className="col col--3">
              <div className="card padding--md">
                <h4>Digital Twin Simulation</h4>
                <p>Creating virtual environments for robot training and testing</p>
              </div>
            </div>
            <div className="col col--3">
              <div className="card padding--md">
                <h4>Isaac AI Brain</h4>
                <p>Advanced perception and navigation using NVIDIA Isaac technologies</p>
              </div>
            </div>
            <div className="col col--3">
              <div className="card padding--md">
                <h4>Physical AI Integration</h4>
                <p>Bringing together hardware and AI for autonomous humanoid robots</p>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* Testimonials Section */}
      <section className="testimonials-section padding-top--lg padding-bottom--lg">
        <div className="container">
          <div className="text--center margin-bottom--lg">
            <h2>What Readers Are Saying</h2>
          </div>
          <div className="row">
            <div className="col col--4">
              <div className="card padding--lg">
                <p>"This book provides the most comprehensive approach to humanoid robotics I've seen. The combination of theory and practical examples is exceptional."</p>
                <div className="avatar margin-top--md">
                  <div className="avatar__intro">
                    <div className="avatar__name">Dr. Sarah Chen</div>
                    <small className="avatar__subtitle">Robotics Researcher</small>
                  </div>
                </div>
              </div>
            </div>
            <div className="col col--4">
              <div className="card padding--lg">
                <p>"Finally, a resource that bridges the gap between AI and physical robotics. The Isaac AI section alone is worth the price of admission."</p>
                <div className="avatar margin-top--md">
                  <div className="avatar__intro">
                    <div className="avatar__name">Marcus Rodriguez</div>
                    <small className="avatar__subtitle">AI Engineer</small>
                  </div>
                </div>
              </div>
            </div>
            <div className="col col--4">
              <div className="card padding--lg">
                <p>"As a student, I found the progression from basic concepts to advanced topics very intuitive. The practical examples helped solidify my understanding."</p>
                <div className="avatar margin-top--md">
                  <div className="avatar__intro">
                    <div className="avatar__name">Aisha Patel</div>
                    <small className="avatar__subtitle">Graduate Student</small>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="cta-section padding-top--xl padding-bottom--xl">
        <div className="container">
          <div className="row">
            <div className="col col--8 col--offset-2 text--center">
              <h2>Start Your Journey in Humanoid Robotics</h2>
              <p className="hero__subtitle">Join thousands of students and professionals mastering the future of AI-powered robotics</p>
              <div className="margin-top--lg">
                <button className="button button--primary button--lg margin-right--md">
                  Begin Reading
                </button>
                <button className="button button--secondary button--lg">
                  View Sample Chapter
                </button>
              </div>
            </div>
          </div>
        </div>
      </section>
    </div>
  );
};

export default LandingPage;