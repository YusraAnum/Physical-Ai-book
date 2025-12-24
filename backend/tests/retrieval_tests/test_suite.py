"""
Comprehensive test suite runner for the retrieval testing functionality.
"""
import pytest
import sys
import os
from pathlib import Path

# Add the src directory to the path so imports work correctly
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def run_retrieval_tests():
    """
    Run the complete retrieval testing suite.
    """
    print("Starting comprehensive retrieval testing suite...")

    # Define test paths
    test_paths = [
        "tests/retrieval_tests/test_retrieval_service.py",
        "tests/retrieval_tests/test_retrieval_api.py",
        "tests/retrieval_tests/test_retrieval_contract.py",
        "tests/retrieval_tests/test_content_verification.py",
        "tests/retrieval_tests/test_content_verification_integration.py",
        "tests/retrieval_tests/test_metadata_validation.py",
        "tests/retrieval_tests/test_metadata_correctness.py",
        "tests/retrieval_tests/test_end_to_end.py",
        "tests/retrieval_tests/test_performance.py",
        "tests/retrieval_tests/test_json_format.py",
        "tests/retrieval_tests/test_comprehensive_pipeline.py",
        "tests/retrieval_tests/test_health.py",
        "tests/retrieval_tests/test_error_handling.py"
    ]

    # Filter to only include tests that exist
    existing_tests = []
    for test_path in test_paths:
        if os.path.exists(test_path):
            existing_tests.append(test_path)
        else:
            print(f"Warning: Test file does not exist: {test_path}")

    if not existing_tests:
        print("No test files found to run!")
        return False

    print(f"Running {len(existing_tests)} test files...")
    print("Test files to run:")
    for test in existing_tests:
        print(f"  - {test}")

    # Run the tests with pytest
    pytest_args = [
        "-v",  # verbose
        "--tb=short",  # short traceback
        *existing_tests
    ]

    print(f"\nRunning pytest with args: {' '.join(pytest_args)}")

    # Execute the tests
    exit_code = pytest.main(pytest_args)

    print(f"\nTest suite completed with exit code: {exit_code}")

    return exit_code == 0

def run_specific_test_groups(groups=None):
    """
    Run specific groups of tests.

    Args:
        groups: List of test groups to run. If None, runs all.
               Options: 'unit', 'integration', 'end-to-end', 'validation', 'health'
    """
    if groups is None:
        groups = ['unit', 'integration', 'end-to-end', 'validation', 'health', 'error']

    print(f"Running test groups: {groups}")

    # Define test group mappings
    test_groups = {
        'unit': [
            "tests/retrieval_tests/test_retrieval_service.py",
            "tests/retrieval_tests/test_content_verification.py",
            "tests/retrieval_tests/test_metadata_validation.py"
        ],
        'integration': [
            "tests/retrieval_tests/test_retrieval_api.py",
            "tests/retrieval_tests/test_content_verification_integration.py",
            "tests/retrieval_tests/test_retrieval_contract.py"
        ],
        'end-to-end': [
            "tests/retrieval_tests/test_end_to_end.py",
            "tests/retrieval_tests/test_comprehensive_pipeline.py"
        ],
        'validation': [
            "tests/retrieval_tests/test_performance.py",
            "tests/retrieval_tests/test_json_format.py"
        ],
        'health': [
            "tests/retrieval_tests/test_health.py"
        ],
        'error': [
            "tests/retrieval_tests/test_error_handling.py"
        ]
    }

    # Collect test files based on requested groups
    test_files = []
    for group in groups:
        if group in test_groups:
            for test_file in test_groups[group]:
                if os.path.exists(test_file):
                    test_files.append(test_file)
                else:
                    print(f"Warning: Test file does not exist: {test_file}")

    if not test_files:
        print("No test files found for the specified groups!")
        return False

    print(f"Running {len(test_files)} test files from groups: {groups}")

    # Run the tests
    pytest_args = [
        "-v",
        "--tb=short",
        *test_files
    ]

    exit_code = pytest.main(pytest_args)

    return exit_code == 0

if __name__ == "__main__":
    print("RAG Retrieval Testing - Test Suite Runner")
    print("=" * 50)

    # Check if specific group is requested
    import argparse
    parser = argparse.ArgumentParser(description="Run RAG retrieval tests")
    parser.add_argument("--group", choices=['unit', 'integration', 'end-to-end', 'validation', 'health', 'error'],
                       help="Run specific test group")
    parser.add_argument("--all", action="store_true", help="Run all tests")

    args = parser.parse_args()

    success = False

    if args.group:
        success = run_specific_test_groups([args.group])
    elif args.all:
        success = run_retrieval_tests()
    else:
        # Default: run all tests
        print("Running all retrieval tests...")
        success = run_retrieval_tests()

    if success:
        print("\n✓ All tests passed!")
        sys.exit(0)
    else:
        print("\n✗ Some tests failed!")
        sys.exit(1)