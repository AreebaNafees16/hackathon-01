# Quickstart Guide

This guide provides instructions for setting up and running the "Physical AI & Humanoid Robotics" project on a local development machine.

## Prerequisites

- **Docker & Docker Compose**: For running the containerized environment.
- **Node.js**: Version 20.x
- **Python**: Version 3.11
- **Git**: For cloning the repository.
- **An OpenAI API Key**: For the chatbot's generation component.

## 1. Environment Setup

1.  **Clone the Repository**:
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```

2.  **Configure Environment Variables**:
    Create a `.env` file in the `backend` directory by copying the example file:
    ```bash
    cp backend/.env.example backend/.env
    ```
    Open `backend/.env` and fill in the required values, especially your `OPENAI_API_KEY`, and credentials for Qdrant and Neon.

## 2. Running the Application

The entire stack (backend, database, frontend) can be launched using Docker Compose.

1.  **Build and Start Services**:
    From the root of the project, run:
    ```bash
    docker-compose up --build
    ```
    This command will:
    - Build the Docker images for the FastAPI backend and the Docusaurus frontend.
    - Start the backend server.
    - Start the frontend development server.
    - (Note: For local dev, this assumes you are running Qdrant/Postgres elsewhere. For a full stack, the docker-compose would include them).

2.  **Access the Services**:
    - **Textbook (Frontend)**: Open your browser and navigate to `http://localhost:3000`.
    - **API (Backend)**: The API will be available at `http://localhost:8000`. You can access the interactive API documentation at `http://localhost:8000/docs`.

## 3. Content Pipeline

To populate the Qdrant vector database with the textbook content, you need to run the content pipeline script.

1.  **Ensure the backend service is running** via `docker-compose up`.

2.  **Execute the processing script**:
    This script will find all markdown files, chunk them, generate embeddings, and upload them to Qdrant.
    ```bash
    docker-compose exec backend python -m scripts.content_pipeline.process
    ```

## 4. Development Workflow

- **Backend Changes**: Modify files in the `backend/src` directory. The `uvicorn` server in the Docker container is configured for hot-reloading, so changes should be applied automatically.
- **Frontend/Content Changes**: Modify markdown files in `frontend/docs` or React components in `frontend/src`. The Docusaurus development server also supports hot-reloading.

## 5. Running Tests

- **Backend Tests**:
  ```bash
  docker-compose exec backend pytest
  ```
- **Frontend Tests**:
  ```bash
  docker-compose exec frontend npm test
  ```
