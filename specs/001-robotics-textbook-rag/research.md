# Research & Decisions

This document records the decisions made to resolve ambiguities identified during the planning phase for the "Physical AI & Humanoid Robotics" project.

## 1. Technology Versioning

- **Decision**:
  - **Python**: 3.11
  - **Node.js**: 20.x
  - **ROS 2**: Humble Hawksbill (LTS)
- **Rationale**: Using recent, stable, and Long-Term Support (LTS) versions where available ensures a good balance of modern features, community support, and stability. This minimizes risks related to bugs or breaking changes in underlying platforms. ROS 2 Humble is the current LTS, making it the most suitable choice for reproducible robotics examples.
- **Alternatives Considered**:
  - Using the absolute latest versions (e.g., Python 3.12, ROS 2 Rolling Ridley): Rejected to avoid the instability and potential for breaking changes inherent in bleeding-edge releases.

## 2. Frontend Framework

- **Decision**: Docusaurus with React.
- **Rationale**: The feature specification explicitly requires Docusaurus-compliant markdown for the textbook. Docusaurus is built on React, making it the natural choice for any custom UI development, such as the chatbot interface. This avoids introducing an unnecessary second frontend framework.
- **Alternatives Considered**:
  - Vue, Svelte: Rejected as they would add complexity by requiring integration with the existing Docusaurus (React) framework.

## 3. Testing Frameworks

- **Decision**:
  - **Backend (Python/FastAPI)**: `pytest`
  - **Frontend (React/Docusaurus)**: `Vitest`
- **Rationale**: `pytest` is the de-facto standard for testing in the Python ecosystem, with extensive plugin support for FastAPI. `Vitest` is a modern, fast, and Jest-compatible test runner that integrates seamlessly with Vite, the build tool used by modern Docusaurus versions.
- **Alternatives Considered**:
  - `unittest` (Python): `pytest` offers a more flexible and less boilerplate-heavy syntax.
  - `Jest` (Frontend): `Vitest` provides better performance and a more modern developer experience with near-complete Jest compatibility.

## 4. RAG Content Chunking Strategy

- **Decision**: Recursive character text splitting with a chunk size of 1000 characters and an overlap of 200 characters. Chapter summaries will be added as metadata to each chunk.
- **Rationale**: A 1000-character chunk size is large enough to contain meaningful semantic context but small enough to fit within most embedding model limits. A 200-character overlap helps preserve context between chunks. Associating the chapter summary as metadata allows for pre-filtering retrieval by topic before performing the semantic search, improving both speed and accuracy.
- **Alternatives Considered**:
  - Fixed-size chunking: Can break sentences and lose semantic meaning.
  - Markdown-aware splitting: More complex to implement; recursive character splitting is a robust and widely-used starting point.

## 5. Deployment Environment

- **Decision**: A Dockerized environment will be defined for local development and CI.
- **Rationale**: To ensure all code examples are reproducible as per the constitution, a `docker-compose.yml` file will be created. This file will define the services, dependencies, and environment variables needed to run the entire system, guaranteeing a consistent environment for all developers and for CI/CD validation.
- **Alternatives Considered**:
  - Manual setup instructions: Prone to "works on my machine" issues and violates the principle of reproducible code.
