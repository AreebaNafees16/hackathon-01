# Implementation Plan: Comprehensive Robotics Textbook and RAG Chatbot

**Branch**: `001-robotics-textbook-rag` | **Date**: 2025-12-11 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/001-robotics-textbook-rag/spec.md`

## Summary

This plan outlines the development of a comprehensive online textbook, "Physical AI & Humanoid Robotics," and an accompanying RAG chatbot. The textbook will consist of 8-12 chapters covering key robotics technologies like ROS 2, Gazebo, and VSLAM. The chatbot, grounded in the textbook's content, will provide users with accurate, citation-backed answers. The final product will be deployed as a Docusaurus site on GitHub Pages with a FastAPI backend.

## Technical Context

**Language/Version**: Python 3.11, Node.js 20.x, ROS 2 Humble Hawksbill (LTS)
**Primary Dependencies**: FastAPI, Docusaurus, OpenAI Python SDK, Qdrant, Neon Postgres, rclpy
**Storage**: Neon Serverless Postgres for relational data (e.g., chat history), Qdrant Cloud for vector embeddings.
**Testing**: `pytest` for backend API testing, `Vitest` for frontend component testing.
**Target Platform**: Web (Linux server for backend, modern browsers for frontend).
**Project Type**: Web Application (Docusaurus frontend, FastAPI backend).
**Performance Goals**: API p95 latency < 300ms; Docusaurus page load (LCP) < 2.5s.
**Constraints**: Chatbot responses MUST be grounded in the textbook content. All code examples MUST be runnable in the specified Dockerized environment.
**Scale/Scope**: ~35k words of content, chatbot designed for low-to-medium query volume.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. High Technical Accuracy**: All technical content is validated against official sources.
- [x] **II. Clear Explanations**: Explanations are clear, consistent, and aimed at the target audience.
- [x] **III. Reproducible Code**: All code and simulations are reproducible.
- [x] **IV. Grounded in Reality**: No hallucinated or unverified APIs, concepts, or facts.
- [x] **V. Content Validation**: All content is validated with official sources.
- [x] **VI. Runnable Code Examples**: Chapters include runnable code in the specified languages.
- [x] **VII. Docusaurus Compliance**: The structure is compatible with Docusaurus.
- [x] **VIII. Grounded Chatbot**: Chatbot responses are strictly based on the book's text.
- [x] **IX. Mandatory Citations**: Major claims are backed by citations.

## Project Structure

### Documentation (this feature)

```text
specs/001-robotics-textbook-rag/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── openapi.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/          # Pydantic data models
│   ├── services/        # Business logic (RAG, content retrieval)
│   ├── api/             # FastAPI endpoints
│   └── core/            # Core configuration, DB connections
└── tests/
    ├── integration/
    └── unit/

frontend/
├── src/
│   ├── components/      # React components (e.g., Chatbot UI)
│   ├── pages/           # Docusaurus page overrides
│   └── theme/           # Docusaurus theme customizations
└── static/
    └── docs/            # Textbook markdown content

scripts/
└── content_pipeline/    # Scripts for chunking, embedding, and loading content
```

**Structure Decision**: A monorepo with distinct `frontend` and `backend` directories is chosen. This provides a clear separation of concerns between the Docusaurus-based textbook/UI and the FastAPI-based chatbot API, while keeping all project code in a single repository for easier management and versioning. The `scripts/content_pipeline` directory will house the data processing scripts for the RAG system.

## Complexity Tracking

No violations of the constitution are anticipated.