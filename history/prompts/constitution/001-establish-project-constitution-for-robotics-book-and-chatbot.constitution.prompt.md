---
id: 001
title: Establish Project Constitution for Robotics Book and Chatbot
stage: constitution
date: 2025-12-11
surface: agent
model: gemini
feature: none
branch: master
user: wajiz.pk
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
  []
---

## Prompt

/sp.constitution
Project:
Book + RAG Chatbot System for “Physical AI & Humanoid Robotics”

Core principles:
- High technical accuracy (ROS 2, Gazebo, Unity, Isaac, VSLAM, Nav2, VLA)
- Consistent, clear explanations for robotics/AI students
- Reproducible code and simulations
- No hallucinated APIs or robotics concepts

Key standards:
- Validate all content with official sources
- Include runnable code (ROS 2 rclpy, Python, FastAPI, JS/TS)
- Docusaurus-compliant structure
- Chatbot must answer only from book or user-selected text
- Citations required for major claims

Constraints:
- Book: 8–12 chapters, ~35k words
- Each chapter must include code, diagrams, and 3+ references
- Use Spec-Kit Plus + Claude Code for generation
- Tech stack:
  - Docusaurus + GitHub Pages
  - FastAPI backend
  - OpenAI Agents/ChatKit SDK
  - Neon Postgres + Qdrant Cloud

Success criteria:
- Book deploys on GitHub Pages without errors
- RAG chatbot fully functional and grounded in book text
- Accurate coverage of all modules (ROS 2, Gazebo/Unity, Isaac, VLA)
- Capstone humanoid robot workflow is implementable end-to-end

## Response snapshot

I have successfully updated the project constitution based on your input.
- Created the initial project constitution at `.specify/memory/constitution.md` (v1.0.0).
- Updated the `.specify/templates/plan-template.md` to include a constitution checklist.
- Verified that no changes were needed for `spec-template.md` or `tasks-template.md`.
- Created this Prompt History Record.

## Outcome

- ✅ Impact: Established the foundational principles and standards for the project. This will guide all future development and ensure consistency.
- 🧪 Tests: No tests were run as this was a documentation and configuration task.
- 📁 Files: 2 files were modified: `.specify/memory/constitution.md` and `.specify/templates/plan-template.md`. 1 file was created: `history/prompts/constitution/001-establish-project-constitution-for-robotics-book-and-chatbot.constitution.prompt.md`.
- 🔁 Next prompts: The user might proceed with `/sp.specify` to define a feature or `/sp.plan` to start architectural planning.
- 🧠 Reflection: The process was smooth. The agent-native fallback for PHR creation worked as expected. Propagating constitution principles to the plan template is a crucial step for ensuring compliance.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
