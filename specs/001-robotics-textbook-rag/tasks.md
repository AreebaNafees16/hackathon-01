# Tasks: Comprehensive Robotics Textbook and RAG Chatbot

**Input**: Design documents from `specs/001-robotics-textbook-rag/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both backend and frontend.

- [ ] T001 Create backend directory structure in `backend/`
- [ ] T002 Create frontend directory structure in `frontend/`
- [ ] T003 Create content pipeline scripts directory in `scripts/content_pipeline/`
- [ ] T004 [P] Initialize Python project with Poetry in `backend/pyproject.toml`
- [ ] T005 [P] Initialize Node.js project and install Docusaurus in `frontend/package.json`
- [ ] T006 [P] Create `backend/.env.example` for environment variables
- [ ] T007 [P] Create `Dockerfile` for the backend service in `backend/Dockerfile`
- [ ] T008 [P] Create `Dockerfile` for the frontend service in `frontend/Dockerfile`
- [ ] T009 Create `docker-compose.yml` to orchestrate backend, frontend, and databases

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [ ] T010 Implement FastAPI app factory and basic configuration in `backend/src/main.py`
- [ ] T011 [P] Implement core database connection module for Neon Postgres in `backend/src/core/db.py`
- [ ] T012 [P] Implement client for Qdrant Cloud in `backend/src/core/qdrant.py`
- [ ] T013 [P] Configure Docusaurus basic settings in `frontend/docusaurus.config.js`
- [ ] T014 [P] Define Pydantic models for TextbookChapter from data-model.md in `backend/src/models/content.py`

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Access Online Textbook (Priority: P1) 🎯 MVP

**Goal**: A student can access the textbook online via a Docusaurus site.

**Independent Test**: The deployed Docusaurus site on GitHub Pages can be accessed, and the user can navigate through chapters.

### Implementation for User Story 1

- [ ] T015 [P] [US1] Create 10 placeholder chapter markdown files in `frontend/docs/`
- [ ] T016 [US1] Configure sidebar navigation for chapters in `frontend/sidebars.js`
- [ ] T017 [P] [US1] Implement API endpoint to list chapters (`/chapters`) in `backend/src/api/content.py`
- [ ] T018 [P] [US1] Implement API endpoint to get a chapter by ID (`/chapters/{chapter_id}`) in `backend/src/api/content.py`
- [ ] T019 [US1] Write unit tests for the content API endpoints in `backend/tests/unit/test_api_content.py`
- [ ] T020 [US1] Create GitHub Actions workflow for Docusaurus deployment to GitHub Pages in `.github/workflows/deploy-docs.yml`

**Checkpoint**: At this point, User Story 1 should be functional. A user can visit the GitHub Pages URL and read placeholder chapter content.

---

## Phase 4: User Story 2 - Query Chatbot for Information (Priority: P2)

**Goal**: A reader can ask the chatbot questions about the book and get grounded, cited answers.

**Independent Test**: The chat interface on the website can be used to ask a question, and a correct, source-based answer is returned.

### Implementation for User Story 2

- [ ] T021 [P] [US2] Define Pydantic models for ChatSession and ChatMessage in `backend/src/models/chat.py`
- [ ] T022 [P] [US2] Implement text chunking script based on research.md in `scripts/content_pipeline/process.py`
- [ ] T023 [US2] Implement embedding generation and upload to Qdrant in `scripts/content_pipeline/process.py`
- [ ] T024 [US2] Implement core RAG retrieval service in `backend/src/services/rag_service.py`
- [ ] T025 [US2] Implement OpenAI Agent generation service in `backend/src/services/generation_service.py`
- [ ] T026 [US2] Implement `/query` API endpoint to integrate services in `backend/src/api/chat.py`
- [ ] T027 [P] [US2] Write unit tests for the RAG and generation services in `backend/tests/unit/`
- [ ] T028 [US2] Write integration test for the `/query` endpoint in `backend/tests/integration/`
- [ ] T029 [P] [US2] Create floating chat button component in `frontend/src/components/ChatButton/`
- [ ] T030 [P] [US2] Create chat window UI component in `frontend/src/components/ChatWindow/`
- [ ] T031 [US2] Implement state management and API client for the chat UI in `frontend/src/components/ChatWindow/`
- [ ] T032 [US2] Integrate the chat components into the Docusaurus theme in `frontend/src/theme/Root.js`

**Checkpoint**: At this point, User Story 2 should be functional. A user can interact with the chatbot on the website.

---

## Phase 5: User Story 3 - Execute Code Examples (Priority: P3)

**Goal**: A developer can easily run and verify code examples from the book.

**Independent Test**: A developer can run a single command to validate all code examples against the specified environment.

### Implementation for User Story 3

- [ ] T033 [US3] Create `examples/` directory for runnable code snippets.
- [ ] T034 [US3] Add a sample ROS 2 publisher/subscriber example to `examples/ros2_chatter/`
- [ ] T035 [US3] Create a validation script to execute all examples in `scripts/validation/run_examples.py`
- [ ] T036 [US3] Update `docker-compose.yml` to include a service for running the validation script.
- [ ] T037 [US3] Document the validation process in `quickstart.md`.

**Checkpoint**: All user stories should now be implemented.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and prepare for release.

- [ ] T038 Add comprehensive logging to all backend services in `backend/src/`
- [ ] T039 [P] Review and add docstrings to all public functions and classes.
- [ ] T040 Perform a final validation of the end-to-end user flow.
- [ ] T041 [P] Update all documentation (`README.md`, `quickstart.md`) for final release.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)** -> **Foundational (Phase 2)** -> **User Stories (Phases 3-5)** -> **Polish (Phase 6)**

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Phase 2.
- **User Story 2 (P2)**: Depends on Phase 2.
- **User Story 3 (P3)**: Depends on Phase 2.

*All user stories can be developed in parallel after Phase 2 is complete.*

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Deploy to GitHub Pages and confirm the placeholder textbook is accessible.

### Parallel Team Strategy

With multiple developers:
1. Team completes Setup + Foundational together.
2. Once Foundational is done:
   - Developer A: Starts on User Story 1 (Textbook content and deployment).
   - Developer B: Starts on User Story 2 (Backend RAG pipeline and API).
   - Developer C: Starts on User Story 2 (Frontend Chat UI).
