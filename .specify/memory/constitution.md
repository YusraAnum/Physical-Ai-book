<!-- SYNC IMPACT REPORT:
Version change: N/A -> 1.0.0
Modified principles: None (new constitution)
Added sections: All sections (new constitution)
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ✅ reviewed
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Simplicity Over Complexity
All implementations must prioritize simplicity and clarity. Solutions should be minimal, well-understood, and avoid unnecessary complications. Every feature addition must justify its existence with clear value proposition.

### Correctness and Structure
Content and code must be factually accurate, properly structured, and minimal in scope. All materials require verification for technical correctness, consistent formatting, and adherence to established standards.

### Free-Tier Compatibility
Architecture must support free-tier service usage for embeddings and API consumption. Resource usage should remain within free tier limits to ensure accessibility and cost-effectiveness for all users.

### Lightweight Design
Development and deployment must avoid heavy computational requirements. Heavy GPU steps are prohibited; prioritize CPU-efficient algorithms and lightweight architectures suitable for edge deployment.

### RAG-Exclusive Content
The integrated chatbot must respond ONLY from the book's text content. No external knowledge sources, hallucinations, or out-of-scope responses are permitted. Strict content fidelity is mandatory.

### Consistent Quality
All content must maintain uniform writing style, formatting, and presentation across chapters and features. Consistency in tone, structure, and user experience is required throughout the textbook and UI.

## Additional Constraints

### Minimal Compute Usage
All features must operate efficiently with minimal computational resources. Algorithms and services should be optimized for speed and memory usage. Heavy processing tasks should be avoided or offloaded appropriately.

### Lightweight Embeddings
Embedding models and processes must remain within free tier usage limits. Choose embedding strategies that balance quality with cost efficiency. Prioritize compact models and efficient vector storage.

### Small Chapter Sizes
Each chapter must remain concise and focused. Content should be broken into digestible sections that maintain reader engagement while covering essential concepts. Large, unwieldy chapters are prohibited.

## Development Workflow

### Docusaurus-Based Structure
The textbook must be built using Docusaurus framework with clean, modern layout. All content should follow Docusaurus conventions and best practices. The UI must be responsive and accessible across devices.

### Integrated RAG Implementation
The RAG system must integrate seamlessly with Qdrant, Neon, and FastAPI. The "select text → ask AI" interaction must work smoothly. All RAG components must be properly tested and documented.

### Feature Readiness
Features like Personalize Chapter, Urdu Translation, and user profile-based content must be properly structured for future implementation. Placeholder components should be clearly marked and ready for extension.

## Governance

The constitution governs all project decisions and supersedes individual team preferences. Amendments require formal documentation and team approval. All pull requests must verify compliance with these principles. Complexity must be thoroughly justified before acceptance.

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
