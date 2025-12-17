import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Comprehensive textbook and RAG chatbot for robotics students">


      <header className={styles.heroBanner}>
        <div className={styles.container}>
          
          <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
          <p className={styles.heroSubtitle}>
             Building intelligent machines that perceive, learn, and interact
          with the physical world — just like humans.
          </p>

          {/* Add image here */}
          <img
            src="/img/ai-hero.jpg"
            alt="Robotics illustration"
            className={styles.heroImage}
          />

          <div className={styles.buttons}>
            <Link className="button button--primary button--lg" to="/docs/basics/intro">
              Start Reading
            </Link>
            <Link className="button button--secondary button--lg" to="/docs/humanoid/capstone">
              Explore Capstone
            </Link>
          </div>
        </div>
      </header>


      <main className={styles.main}>


      {/* INTRO SECTION */}
      <section className={styles.section}>
        <h2>What is Physical AI?</h2>
        <p className={styles.text}>
          Physical AI represents the next evolution of artificial intelligence —
          systems that are not limited to screens or simulations, but are embodied
          in real-world machines. These systems combine perception, reasoning,
          learning, and action to operate safely and autonomously in dynamic
          environments.
        </p>
      </section>

      {/* HUMANOID ROBOTICS */}
      <section className={styles.sectionAlt}>
        <h2>Humanoid Robotics</h2>
        <p className={styles.text}>
          Humanoid robots are designed to mirror human form and behavior,
          allowing them to work seamlessly in environments built for people.
          From walking and grasping to communication and collaboration,
          humanoid robotics pushes the boundary of what machines can achieve.
        </p>

        <div className={styles.featuresGrid}>
          <div className={styles.featureCard}>
            <h3>Human-Like Motion</h3>
            <p>Advanced locomotion, balance, and manipulation.</p>
          </div>

          <div className={styles.featureCard}>
            <h3>Perception & Sensors</h3>
            <p>Vision, touch, audio, and environmental awareness.</p>
          </div>

          <div className={styles.featureCard}>
            <h3>Adaptive Learning</h3>
            <p>Learning from interaction and real-world feedback.</p>
          </div>

          <div className={styles.featureCard}>
            <h3>Safe Interaction</h3>
            <p>Designed to collaborate safely with humans.</p>
          </div>
        </div>
      </section>

      {/* WHY IT MATTERS */}
      <section className={styles.section}>
        <h2>Why Physical AI Matters</h2>

        <div className={styles.impactGrid}>
          <div className={styles.impactCard}>Healthcare & Rehabilitation</div>
          <div className={styles.impactCard}>Manufacturing & Automation</div>
          <div className={styles.impactCard}>Disaster Response</div>
          <div className={styles.impactCard}>Education & Research</div>
        </div>
      </section>

      {/* FOCUS AREAS */}
      <section className={styles.sectionAlt}>
        <h2>Core Focus Areas</h2>

        <ul className={styles.focusList}>
          <li>Embodied Intelligence & Decision Making</li>
          <li>Robot Control Systems & Dynamics</li>
          <li>Human-Robot Interaction (HRI)</li>
          <li>AI-Driven Perception & Sensor Fusion</li>
          <li>Autonomous Navigation & Manipulation</li>
        </ul>
      </section>
        
       
        <section className={styles.ctaSection}>
          <div className={styles.container}>
            <h2>Get Started Today!</h2>
            <p>Start reading the textbook or try the interactive RAG Chatbot to learn robotics practically.</p>
            <div className={styles.buttons}>
              <Link className="button button--primary button--lg" to="/docs/basics/intro">
                Start Reading
              </Link>
              <Link className="button button--success button--lg" to="/chatbot">
                Try the RAG Chatbot
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
