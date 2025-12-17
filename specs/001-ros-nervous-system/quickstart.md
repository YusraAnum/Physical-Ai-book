# Quickstart Guide: ROS 2 as a Robotic Nervous System Book

**Feature**: Technical Book Built with Docusaurus
**Branch**: 001-ros-nervous-system
**Date**: 2025-12-15

## Overview

This quickstart guide provides the essential steps to set up, build, and contribute to the "ROS 2 as a Robotic Nervous System" technical book built with Docusaurus. The book focuses on ROS 2 middleware concepts for humanoid robotics applications.

## Prerequisites

Before getting started, ensure you have the following installed:

- **Node.js**: Version 18.x or higher
- **npm**: Version 8.x or higher (or Yarn 1.22+)
- **Git**: Version control system
- **Text Editor**: VS Code, Vim, or your preferred editor

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
npm install
# OR if using Yarn
yarn install
```

### 3. Start Local Development Server

```bash
npm start
# OR if using Yarn
yarn start
```

This command starts a local development server and opens the book in your browser at `http://localhost:3000`. Most changes are reflected live without restarting the server.

### 4. Build the Static Site

```bash
npm run build
# OR if using Yarn
yarn run build
```

This command generates static content into the `build` directory, which can be served using any static hosting service.

### 5. Serve the Built Site Locally

```bash
npm run serve
# OR if using Yarn
yarn run serve
```

This command serves the built static content for testing purposes.

## Project Structure

```
book/
├── docs/
│   ├── intro.md
│   ├── chapter-1/
│   │   ├── middleware-concept.md
│   │   ├── ros2-role.md
│   │   └── architecture-comparison.md
│   ├── chapter-2/
│   │   ├── nodes-topics-services.md
│   │   ├── message-passing.md
│   │   └── control-signals.md
│   └── chapter-3/
│       ├── python-ros-integration.md
│       ├── rclpy-examples.md
│       └── urdf-introduction.md
├── src/
│   ├── components/
│   ├── pages/
│   └── css/
├── static/
│   ├── img/
│   └── assets/
├── docusaurus.config.js
├── sidebars.js
├── package.json
└── README.md
```

## Content Creation

### Adding New Content

1. **Create a new markdown file** in the appropriate chapter directory:
   ```bash
   # Example: Adding a new section to Chapter 1
   touch docs/chapter-1/new-section.md
   ```

2. **Add frontmatter** to your markdown file:
   ```markdown
   ---
   title: New Section Title
   sidebar_position: 3
   description: Brief description of the section
   ---

   # New Section Title

   Your content here...
   ```

3. **Update the sidebar** in `sidebars.js` to include your new page:
   ```javascript
   module.exports = {
     docs: [
       // ... existing content
       {
         type: 'category',
         label: 'Chapter 1: ROS 2 as Middleware',
         items: [
           'chapter-1/middleware-concept',
           'chapter-1/ros2-role',
           'chapter-1/architecture-comparison',
           'chapter-1/new-section'  // Add your new section here
         ],
       },
       // ... rest of the sidebar
     ],
   };
   ```

### Writing Content

#### Basic Markdown
Use standard markdown syntax for text formatting:
- Headers: `# Main Header`, `## Sub Header`, etc.
- Bold: `**bold text**`
- Italic: `*italic text*`
- Lists: `- item` for unordered, `1. item` for ordered
- Links: `[text](url)`
- Images: `![alt text](path/to/image)`

#### Docusaurus-Specific Features

**Admonitions** (callout blocks):
```markdown
:::note
This is a note.
:::

:::tip
This is a tip.
:::

:::caution
This is a caution.
:::

:::danger
This is a danger.
:::
```

**Code Blocks**:
```markdown
// With syntax highlighting
\```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
\```

// With title and line highlighting
\```python title="publisher_member_function.py" {1,4-6}
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
\```
```

**Math Equations** (if MathJax is enabled):
```markdown
Inline: $x = y + z$

Block:
$$
\int_{-\infty}^{\infty} e^{-x^2} dx = \sqrt{\pi}
$$
```

### Adding Figures and Diagrams

1. **Place images** in the `static/img/` directory
2. **Reference them** in your markdown:
   ```markdown
   ![ROS 2 Architecture Diagram](/img/ros2-architecture.png)
   ```

3. **Use relative paths** from the static directory

### Adding Code Examples

1. **Inline code**: Use single backticks: `code here`
2. **Code blocks**: Use triple backticks with language specification
3. **External files**: Reference external code files:
   ```markdown
   \```python title="src/example.py"
   {/path/to/external/file.py}
   \```

## Configuration

### Site Configuration (`docusaurus.config.js`)

Key configuration options:

- **title**: Site title
- **tagline**: Site tagline
- **url**: Site URL
- **baseUrl**: Base URL for the site
- **favicon**: Path to favicon
- **organizationName**: GitHub organization name (for deployment)
- **projectName**: GitHub project name (for deployment)

### Sidebar Configuration (`sidebars.js`)

Organize your content in the sidebar:

```javascript
module.exports = {
  docs: [
    'intro',  // Single page
    {
      type: 'category',  // Collapsible section
      label: 'Chapter 1',
      items: [
        'chapter-1/page1',
        'chapter-1/page2'
      ],
    },
  ],
};
```

## Validation and Quality Checks

### Content Validation

1. **Build validation**:
   ```bash
   npm run build
   ```

2. **Link validation**: Docusaurus automatically checks for broken links

3. **Markdown linting** (if configured):
   ```bash
   npm run lint
   ```

### Technical Accuracy Checks

- Verify all code examples work as described
- Cross-reference with official ROS 2 documentation
- Ensure technical concepts are accurately represented
- Use credible sources for all claims (follow APA citation style)

### Citation Format

Use APA style for in-text citations:
- (Author, Year) for general references
- (Author, Year, p. X) for direct quotes or specific pages

Example:
```markdown
According to Smith (2023), ROS 2 implements a data distribution service (p. 45).
```

## Deployment

### GitHub Pages

1. Set up your GitHub repository with GitHub Pages enabled
2. Configure the following in `docusaurus.config.js`:
   ```javascript
   module.exports = {
     // ...
     organizationName: 'your-github-username',
     projectName: 'your-repo-name',
     deploymentBranch: 'gh-pages',
     // ...
   };
   ```

3. Deploy using:
   ```bash
   npm run deploy
   ```

### Other Platforms

For other platforms (Netlify, Vercel, etc.), build the site and upload the contents of the `build` directory.

## Troubleshooting

### Common Issues

**Port already in use**:
```bash
npm start -- --port 3001
```

**Clear cache**:
```bash
npm start -- --clear-cache
```

**Build fails with memory error**:
```bash
export NODE_OPTIONS="--max_old_space_size=4096"
npm run build
```

### Development Tips

- Use `npm run serve` to test the production build locally
- Enable hot reloading during development with `npm start`
- Use the Docusaurus debug plugin to inspect site data
- Check the browser console for any client-side errors

## Next Steps

1. Review the existing content structure in `docs/`
2. Follow the content creation guidelines above to add new material
3. Test your changes locally before committing
4. Follow the APA citation style for all references
5. Ensure all technical content is verified against official ROS 2 documentation