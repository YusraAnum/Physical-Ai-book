# Data Model: ROS 2 as a Robotic Nervous System Book

**Feature**: Technical Book Built with Docusaurus
**Branch**: 001-ros-nervous-system
**Date**: 2025-12-15

## Content Entities

### Book
- **name**: string - Title of the book
- **subtitle**: string - Subtitle or description
- **version**: string - Version identifier
- **authors**: array of strings - Book authors
- **description**: string - Book description
- **target_audience**: string - Intended audience (e.g., "Undergraduate CS/Robotics students")
- **total_word_count**: integer - Total word count of the book
- **chapters**: array of Chapter - List of chapters in the book
- **references**: array of Reference - List of all references used in the book
- **metadata**: object - Additional metadata (publication date, license, etc.)

### Chapter
- **id**: string - Unique identifier for the chapter
- **title**: string - Chapter title
- **subtitle**: string - Chapter subtitle (optional)
- **word_count**: integer - Number of words in the chapter
- **sections**: array of Section - List of sections within the chapter
- **learning_objectives**: array of string - What the reader should learn from this chapter
- **prerequisites**: array of string - Knowledge required before reading this chapter
- **difficulty_level**: enum (beginner, intermediate, advanced) - Difficulty level of the chapter
- **estimated_reading_time**: integer - Estimated time in minutes to read the chapter

### Section
- **id**: string - Unique identifier for the section
- **title**: string - Section title
- **content**: string - The main content of the section in markdown format
- **type**: enum (conceptual, technical, practical, example) - Type of content
- **figures**: array of Figure - List of figures used in the section
- **code_examples**: array of CodeExample - List of code examples in the section
- **references**: array of Reference - References used in this section
- **learning_outcomes**: array of string - Specific outcomes from this section

### Figure
- **id**: string - Unique identifier for the figure
- **title**: string - Title or caption for the figure
- **description**: string - Description of the figure content
- **file_path**: string - Path to the image file
- **alt_text**: string - Alternative text for accessibility
- **type**: enum (diagram, chart, screenshot, illustration) - Type of figure
- **position**: enum (inline, center, full-width) - How to display the figure

### CodeExample
- **id**: string - Unique identifier for the code example
- **title**: string - Title or description of the example
- **language**: string - Programming language (e.g., "python", "xml", "bash")
- **code**: string - The actual code content
- **explanation**: string - Explanation of what the code does
- **file_path**: string - Path to external code file if applicable
- **line_numbers**: boolean - Whether to show line numbers

### Reference
- **id**: string - Unique identifier for the reference
- **type**: enum (book, journal_article, website, documentation, conference_paper) - Type of source
- **author**: string - Author or organization name
- **title**: string - Title of the work
- **year**: integer - Publication year
- **publisher**: string - Publisher name
- **url**: string - URL if available (optional)
- **doi**: string - DOI if available (optional)
- **page_numbers**: string - Page range if applicable
- **apa_citation**: string - Full APA format citation
- **accessed_date**: string - Date when source was accessed (for web sources)

## Relationships

- Book **has many** Chapters (1 to many)
- Chapter **has many** Sections (1 to many)
- Section **has many** Figures (1 to many)
- Section **has many** CodeExamples (1 to many)
- Section **has many** References (1 to many)
- Book **has many** References (1 to many) - for global references

## Validation Rules

### Book Level
- Total word count must be between 2,000 and 3,000 words
- Must have exactly 3 chapters as specified in requirements
- Target audience must be "Undergraduate CS/Robotics students and early-stage robotics developers"
- Title must include "ROS 2" and "Robotic Nervous System"

### Chapter Level
- Chapter title must be descriptive and related to ROS 2 concepts
- Word count per chapter should be balanced across all chapters
- Must have at least one learning objective
- Difficulty level must be appropriate for target audience (beginner to intermediate)
- Estimated reading time should be realistic (15-45 minutes per chapter)

### Section Level
- Content must be in valid markdown format
- Type must be one of the defined enum values
- If type is "example", must include at least one CodeExample
- All referenced figures and code examples must exist

### Reference Level
- All references must follow APA citation style
- URL must be valid if provided
- Year must be within the last 10 years for technical accuracy
- Must have at least 2 credible sources per major concept as specified in requirements

## State Transitions

### Content Creation Workflow
1. **Draft** → Content is being written
2. **Reviewed** → Content has been reviewed for technical accuracy
3. **Validated** → Content meets all requirements and passes validation checks
4. **Published** → Content is ready for deployment

### Validation States
- **Pending**: Awaiting validation
- **Valid**: Meets all validation rules
- **Invalid**: Fails one or more validation rules with specific error messages

## Indexes and Search Considerations

- Full-text search on content fields (title, content, explanations)
- Index by chapter and section for navigation
- Tag-based search for concepts (middleware, ROS2, Nodes, Topics, Services, etc.)
- Author and year indexes for reference management