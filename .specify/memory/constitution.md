<!--
---
Sync Impact Report
---
- Version: 0.0.0 → 1.0.0
- Added Principles:
  - High Technical Accuracy
  - Clear Explanations
  - Reproducible Code
  - Grounded in Reality
  - Content Validation
  - Runnable Code Examples
  - Docusaurus Compliance
  - Grounded Chatbot
  - Mandatory Citations
- Added Sections:
  - Key Project Constraints
  - Success Criteria
- Templates requiring updates:
  - ⚠ .specify/templates/plan-template.md
  - ⚠ .specify/templates/spec-template.md
  - ⚠ .specify/templates/tasks-template.md
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Set initial adoption date.
-->

# Book + RAG Chatbot System for “Physical AI & Humanoid Robotics” Constitution

## Core Principles

### I. High Technical Accuracy
All technical content must be accurate and validated against official documentation and sources for technologies including ROS 2, Gazebo, Unity, Isaac Sim, VSLAM, Nav2, and VLAs.

### II. Clear Explanations
Explanations of robotics and AI concepts must be consistent, clear, and targeted at a student audience.

### III. Reproducible Code
All code and simulation examples provided must be reproducible by readers.

### IV. Grounded in Reality
The system must not produce hallucinated APIs, libraries, or robotics concepts. All information must be verifiable.

### V. Content Validation
All content must be validated with official sources.

### VI. Runnable Code Examples
Each chapter must include runnable code examples using ROS 2 (rclpy), Python, FastAPI, and/or JavaScript/TypeScript.

### VII. Docusaurus Compliance
The project structure and content must be compliant with Docusaurus for proper rendering and deployment.

### VIII. Grounded Chatbot
The RAG chatbot must only answer questions based on the content of the book or user-selected text.

### IX. Mandatory Citations
Citations are required for all major claims and technical assertions.

## Key Project Constraints

- **Book Structure:** The book will consist of 8–12 chapters, with a total word count of approximately 35,000 words.
- **Chapter Content:** Each chapter must include runnable code, diagrams, and a minimum of three references.
- **Generation Tools:** The primary tools for content generation will be Spec-Kit Plus and Claude Code.
- **Technology Stack:**
  - **Frontend/Deployment:** Docusaurus and GitHub Pages
  - **Backend API:** FastAPI
  - **Chatbot Framework:** OpenAI Agents/ChatKit SDK
  - **Database:** Neon Postgres and Qdrant Cloud

## Success Criteria

- **Deployment:** The book must deploy successfully to GitHub Pages without any build or rendering errors.
- **Chatbot Functionality:** The RAG chatbot must be fully functional and its responses strictly grounded in the book's text.
- **Technical Coverage:** The content must provide accurate and comprehensive coverage of all specified modules (ROS 2, Gazebo/Unity, Isaac Sim, VLA).
- **Capstone Project:** The final capstone project, a humanoid robot workflow, must be implementable from end-to-end based on the book's guidance.

## Governance

Amendments to this constitution require documented discussion, a clear rationale, and an approved migration plan for any affected project components. All development activities, reviews, and automated checks must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Set initial adoption date. | **Last Amended**: 2025-12-11