---
id: 3
title: set docs routeBasePath and baseUrl
stage: general
date: '2025-12-10'
model: Claude Sonnet 4.5
feature: none
branch: 001-physical-ai-book
user: Claude
command: /sp.implement physical-ai-book --tasks "set docs routeBasePath to '/', set baseUrl to '/physical-ai-book/'"
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

# set docs routeBasePath and baseUrl

## User Prompt
```
physical-ai-book --tasks "set docs routeBasePath to '/', set baseUrl to '/physical-ai-book/'
```

## Response
Successfully updated the Docusaurus configuration with the requested changes:

1. Kept `routeBasePath: '/'` in the docs configuration (was already set correctly)
2. Updated `baseUrl` from '/' back to '/physical-ai-book/' as requested
3. Updated the logo href to point to '/physical-ai-book/' to match the new baseUrl
4. Verified that the Docusaurus build completes successfully with the new configuration

The documentation will continue to be served from the root of the base URL path (meaning docs are accessible at /physical-ai-book/ rather than /physical-ai-book/docs/), while maintaining the correct base path for deployment.

## Outcome
The Docusaurus configuration now has routeBasePath set to '/' and baseUrl set to '/physical-ai-book/' as requested. The build completes successfully with these settings.