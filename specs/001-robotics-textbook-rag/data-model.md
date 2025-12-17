# Data Model

This document defines the key data entities for the "Physical AI & Humanoid Robotics" textbook and RAG chatbot system.

## 1. TextbookChapter

Represents a single chapter in the book. This entity is primarily defined by the markdown file structure but will be represented in the database for content management and retrieval.

- **`id`**: `UUID` - Unique identifier for the chapter.
- **`chapter_number`**: `Integer` - The sequential number of the chapter (e.g., 1, 2, 3).
- **`title`**: `String` - The title of the chapter.
- **`content_markdown`**: `Text` - The full markdown content of the chapter.
- **`summary`**: `Text` - A concise summary of the chapter, used for RAG indexing and providing context to the LLM.
- **`references`**: `JSON` - A JSON array of reference strings formatted in IEEE style.
  - Example: `["[1] S. M. LaValle, Planning Algorithms. Cambridge University Press, 2006."]`
- **`diagrams`**: `JSON` - A JSON array of paths or identifiers for diagrams within the chapter.
  - Example: `["/img/chapter1/vla-architecture.png"]`
- **`code_snippets`**: `JSON` - A JSON array of code snippet identifiers or metadata.

## 2. VectorChunk (in Qdrant)

Represents a chunk of text from the textbook stored as a vector in the Qdrant database for semantic search.

- **`id`**: `UUID` - Unique identifier for the vector.
- **`vector`**: `Float[]` - The high-dimensional vector embedding of the content.
- **`payload`**: `JSON` - Metadata associated with the vector.
  - **`content`**: `String` - The raw text content of the chunk.
  - **`chapter_id`**: `UUID` - Foreign key linking to the `TextbookChapter`.
  - **`chapter_title`**: `String` - Title of the source chapter for quick reference.
  - **`page_number`**: `Integer` - (Optional) Page number or location within the chapter.

## 3. ChatSession (in Neon Postgres)

Represents a single user's interaction session with the chatbot.

- **`id`**: `UUID` - Unique identifier for the session.
- **`user_id`**: `String` - An identifier for the user (can be an anonymous session ID).
- **`start_time`**: `Timestamp` - The time the session began.
- **`end_time`**: `Timestamp` (nullable) - The time the session ended.
- **`metadata`**: `JSON` - Any additional metadata about the session.

## 4. ChatMessage (in Neon Postgres)

Represents a single message within a chat session, from either the user or the chatbot.

- **`id`**: `UUID` - Unique identifier for the message.
- **`session_id`**: `UUID` - Foreign key linking to the `ChatSession`.
- **`timestamp`**: `Timestamp` - The time the message was created.
- **`is_user_message`**: `Boolean` - `true` if the message is from the user, `false` if from the chatbot.
- **`content`**: `Text` - The text of the message.
- **`citations`**: `JSON` (nullable) - For chatbot messages, a JSON array of source citations used to generate the answer.
  - Example: `[{"chapter_id": "...", "text_snippet": "..."}]`
