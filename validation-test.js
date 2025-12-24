/**
 * Quickstart Validation Test for RAG Chat Integration
 * This script validates that the RAG chat functionality works as expected
 */

const { spawn } = require('child_process');
const fetch = require('node-fetch');

console.log('Starting RAG Chat Integration validation...');

// Test 1: Check if backend is running
async function testBackendHealth() {
  try {
    const response = await fetch('http://localhost:8000/api/rag/health');
    const data = await response.json();

    if (response.ok && data.status === 'healthy') {
      console.log('✅ Backend health check: PASSED');
      return true;
    } else {
      console.log('❌ Backend health check: FAILED');
      console.log('Response:', data);
      return false;
    }
  } catch (error) {
    console.log('❌ Backend health check: FAILED - Cannot connect to backend');
    console.log('Error:', error.message);
    return false;
  }
}

// Test 2: Test query functionality
async function testQueryFunctionality() {
  try {
    const response = await fetch('http://localhost:8000/api/rag/query', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: 'Test query',
        context: 'Test context',
        session_id: 'test-session-123'
      })
    });

    const data = await response.json();

    if (response.ok && data.status) {
      console.log('✅ Query functionality: PASSED');
      console.log('Response status:', data.status);
      return true;
    } else {
      console.log('❌ Query functionality: FAILED');
      console.log('Response:', data);
      return false;
    }
  } catch (error) {
    console.log('❌ Query functionality: FAILED - Cannot connect to backend');
    console.log('Error:', error.message);
    return false;
  }
}

// Test 3: Check if frontend components exist
function testFrontendComponents() {
  const fs = require('fs');
  const path = require('path');

  const frontendComponents = [
    'book-physical-ai-humanoid-robotics/src/components/ChatInterface/ChatWindow.js',
    'book-physical-ai-humanoid-robotics/src/components/ChatInterface/Message.js',
    'book-physical-ai-humanoid-robotics/src/components/ChatInterface/InputArea.js',
    'book-physical-ai-humanoid-robotics/src/services/rag-api.js',
    'book-physical-ai-humanoid-robotics/static/js/text-selection.js'
  ];

  let allExist = true;
  for (const component of frontendComponents) {
    if (fs.existsSync(component)) {
      console.log(`✅ Frontend component exists: ${component}`);
    } else {
      console.log(`❌ Frontend component missing: ${component}`);
      allExist = false;
    }
  }

  return allExist;
}

// Run validation tests
async function runValidation() {
  console.log('\nRunning validation tests...\n');

  const backendHealth = await testBackendHealth();
  console.log('');

  const queryTest = await testQueryFunctionality();
  console.log('');

  const frontendTest = testFrontendComponents();
  console.log('');

  const allPassed = backendHealth && queryTest && frontendTest;

  if (allPassed) {
    console.log('🎉 All validation tests PASSED!');
    console.log('RAG Chat Integration is working correctly.');
  } else {
    console.log('❌ Some validation tests FAILED!');
    console.log('Please check the implementation.');
  }

  process.exit(allPassed ? 0 : 1);
}

// Run the validation
runValidation().catch(error => {
  console.error('Validation error:', error);
  process.exit(1);
});