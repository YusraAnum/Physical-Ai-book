# Quickstart: RAG Chat Integration

## Overview
This guide provides instructions for setting up and running the RAG chat integration feature in the Physical AI & Humanoid Robotics book.

## Prerequisites
- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Git
- Basic knowledge of React and FastAPI

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Start the Backend Service
```bash
cd backend
pip install -r requirements.txt
python -m src.main
```
The backend service will start on `http://localhost:8000`

### 3. Start the Docusaurus Frontend
```bash
cd book-physical-ai-humanoid-robotics
npm install
npm start
```
The frontend will start on `http://localhost:3000`

### 4. Verify Integration
1. Navigate to any book page in the browser
2. Look for the chat interface widget (typically in the bottom-right corner)
3. Test functionality by:
   - Submitting a general question about book content
   - Selecting text and asking a question about the selection
   - Verifying loading states appear during processing
   - Testing error handling (e.g., by temporarily stopping the backend)

## Development Commands

### Frontend Development
```bash
# Start development server
npm start

# Build for production
npm run build

# Run tests
npm test
```

### Backend Development
```bash
# Install dependencies
pip install -r requirements.txt

# Start development server
python -m src.main

# Run tests
python -m pytest tests/
```

## Key Components

### Frontend Components
- `ChatInterface/ChatWindow.js`: Main chat interface component
- `ChatInterface/Message.js`: Individual message display component
- `ChatInterface/InputArea.js`: Input area with submission handling
- `services/rag-api.js`: API service for communicating with backend
- `static/js/text-selection.js`: Text selection functionality

### Backend Endpoints
- `POST /api/rag/query`: Submit queries and receive responses
- `GET /api/rag/health`: Health check endpoint

## Configuration
The integration is configured to work with the local development environment by default. No additional configuration is required for basic functionality.

## Troubleshooting
- If the chat interface doesn't appear, verify the backend is running on `http://localhost:8000`
- If queries fail, check browser console for CORS or network errors
- Ensure the backend RAG service is properly configured with book embeddings