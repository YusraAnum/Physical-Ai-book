# Data Model: RAG Chat Integration

## Entities

### User Query
- **id**: string (unique identifier for the query)
- **content**: string (the text of the user's question)
- **context**: string (optional selected text that provides context for the query)
- **timestamp**: datetime (when the query was submitted)
- **type**: enum (GENERAL_QUESTION | SELECTED_TEXT_QUESTION)

### RAG Response
- **id**: string (unique identifier for the response, matching the query id)
- **content**: string (the AI-generated response text)
- **sources**: array of Source objects (references to book content that informed the response)
- **timestamp**: datetime (when the response was received)
- **status**: enum (SUCCESS | ERROR | EMPTY)

### Source
- **id**: string (identifier for the source document/chapter/section)
- **title**: string (title of the source)
- **content**: string (relevant excerpt from the source)
- **page_reference**: string (page number or section reference)

### Chat Session
- **id**: string (unique identifier for the session)
- **user_queries**: array of User Query objects (queries in this session)
- **responses**: array of RAG Response objects (responses in this session)
- **created_at**: datetime (when the session was started)
- **last_activity**: datetime (when the last interaction occurred)

### API Request Payload
- **query**: string (the user's question)
- **context**: string (optional context from selected text)
- **session_id**: string (optional session identifier to maintain conversation context)

### API Response Payload
- **response**: string (the AI-generated response)
- **sources**: array of Source objects (sources used to generate the response)
- **status**: enum (SUCCESS | ERROR | EMPTY)
- **error_message**: string (optional error message if status is ERROR)

## State Objects

### Chat UI State
- **isLoading**: boolean (indicates if a response is being fetched)
- **currentQuery**: string (the current user input)
- **messages**: array of Message objects (chat history displayed in UI)
- **error**: string | null (error message if any)
- **isChatOpen**: boolean (whether the chat interface is visible)

### Message Object
- **id**: string (unique identifier)
- **content**: string (message text)
- **sender**: enum (USER | ASSISTANT)
- **timestamp**: datetime (when the message was created)
- **sources**: array of Source objects (for assistant messages)