---
id: 2
title: set docs routeBasePath to root
stage: general
date: '2025-12-10'
model: Claude Sonnet 4.5
feature: none
branch: 001-physical-ai-book
user: Claude
command: /sp.implement physical-ai-book --tasks "set docs routeBasePath to '/'"
labels:
  - docusaurus
  - configuration
  - routing
files:
  - docusaurus.config.js
tests: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
---

# set docs routeBasePath to root

## User Prompt
```
physical-ai-book --tasks "set docs routeBasePath to '/'"
```

## Response
Successfully updated the Docusaurus configuration to serve docs from the root path:

1. Modified `docusaurus.config.js` to set `routeBasePath: '/'` in the docs configuration
2. Updated `baseUrl` from `/physical-ai-book` to `/` to serve the entire site from root
3. Updated footer navigation links to reflect the new route structure (removed `/docs/` prefix)
4. Updated logo href to point to root (`/`) instead of `/docs/`
5. Verified that the Docusaurus build completes successfully with the new configuration

The documentation will now be served from the root path instead of under a `/docs/` subdirectory, making the URLs cleaner and more accessible.

## Outcome
The documentation site now serves all content from the root path as requested. The routeBasePath has been set to '/' allowing docs to be accessed directly without the '/docs/' prefix.