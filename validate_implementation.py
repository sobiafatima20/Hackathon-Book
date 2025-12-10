#!/usr/bin/env python3
"""
Validation script for the Physical AI & Humanoid Robotics Technical Book implementation.
This script checks if all required files and directories exist as specified in the plan.
"""

import os
import sys
from pathlib import Path


def check_directory_exists(path):
    """Check if a directory exists."""
    return Path(path).is_dir()


def check_file_exists(path):
    """Check if a file exists."""
    return Path(path).is_file()


def check_for_nested_docusaurus_configs():
    """Check for potential nested Docusaurus configurations that might confuse the build system."""
    print("\nChecking for potential Docusaurus configuration conflicts...")

    # Look for any additional config files that might conflict with the main ones
    config_patterns = [
        "docusaurus.config.*",
        "sidebars.*",
        "docusaurus.*",
        "*docusaurus*"
    ]

    # Find all potential config files (excluding the main expected ones and build artifacts)
    potential_conflicts = []
    for root, dirs, files in os.walk("."):
        # Skip node_modules and .docusaurus (which is expected build directory)
        dirs[:] = [d for d in dirs if d not in ['.docusaurus', 'node_modules', '.git']]

        for file in files:
            if any(pattern.replace("*", "") in file.lower() for pattern in config_patterns) and \
               file not in ['docusaurus.config.js', 'sidebars.js']:
                full_path = Path(root) / file
                if str(full_path) != 'docusaurus.config.js' and str(full_path) != 'sidebars.js':
                    potential_conflicts.append(str(full_path))

    if potential_conflicts:
        print("⚠️  Potential configuration conflicts found:")
        for conflict in potential_conflicts:
            print(f"  - {conflict}")
        print("\n  Note: Only docusaurus.config.js and sidebars.js should exist at the project root.")
        print("  Nested configurations can confuse the build system.")
        return False
    else:
        print("  ✓ No conflicting Docusaurus configurations found")
        return True


def validate_project_structure():
    """Validate the project structure according to the implementation plan."""
    print("Validating Physical AI & Humanoid Robotics Technical Book implementation...")

    # Required directories
    required_dirs = [
        "docs",
        "docs/modules",
        "docs/modules/ros2-nervous-system",
        "docs/modules/digital-twin",
        "docs/modules/ai-robot-brain",
        "docs/modules/vla-system",
        "docs/modules/ros2-nervous-system/labs",
        "docs/modules/digital-twin/labs",
        "docs/modules/ai-robot-brain/labs",
        "docs/modules/vla-system/labs",
        "docs/reference",
        "docs/src",
        "docs/src/pages",
        "book",
        "book/ros2-workspace",
        "book/ros2-workspace/src",
        "book/ros2-workspace/src/humanoid_description",
        "book/ros2-workspace/src/ros2_nervous_system_examples",
        "book/ros2-workspace/src/ai_agent",
        "book/gazebo-models",
        "book/gazebo-models/worlds",
        "book/gazebo-models/models",
        "book/gazebo-models/plugins",
        "book/unity-scenes",
        "book/unity-scenes/humanoid",
        "book/unity-scenes/environments",
        "book/unity-scenes/ros2_bridge",
        "book/isaac-configs",
        "book/isaac-configs/perception",
        "book/isaac-configs/navigation",
        "book/isaac-configs/synthetic_data",
        "book/vla-examples",
        "book/vla-examples/asr",
        "book/vla-examples/llm",
        "book/vla-examples/planning"
    ]

    # Required files
    required_files = [
        "README.md",
        "package.json",
        "docusaurus.config.js",
        "sidebars.js",
        ".gitignore",
        "requirements.txt",
        "docs/index.md",
        "docs/modules/ros2-nervous-system/index.md",
        "docs/modules/digital-twin/index.md",
        "docs/modules/ai-robot-brain/index.md",
        "docs/modules/vla-system/index.md",
        "docs/reference/glossary.md",
        "docs/reference/citations.md",
        "book/ros2-workspace/src/humanoid_description/urdf/humanoid.urdf",
        "book/ros2-workspace/src/humanoid_description/package.xml",
        "book/ros2-workspace/src/humanoid_description/CMakeLists.txt",
        "book/ros2-workspace/src/ros2_nervous_system_examples/package.xml",
        "book/ros2-workspace/src/ros2_nervous_system_examples/setup.py",
        "book/ros2-workspace/src/ros2_nervous_system_examples/ros2_nervous_system_examples/talker.py",
        "book/ros2-workspace/src/ros2_nervous_system_examples/ros2_nervous_system_examples/listener.py",
        "book/ros2-workspace/src/ai_agent/package.xml",
        "book/ros2-workspace/src/ai_agent/setup.py",
        "book/ros2-workspace/src/ai_agent/ai_agent/simple_ai_agent.py",
        "book/isaac-configs/perception/perception_pipeline_config.yaml",
        "book/isaac-configs/navigation/nav2_config.yaml",
        "book/vla-examples/asr/asr_config.yaml",
        "book/vla-examples/llm/llm_config.yaml",
        "book/vla-examples/planning/action_planning_config.yaml"
    ]

    # Module content files
    module_files = [
        "docs/modules/ros2-nervous-system/architecture.md",
        "docs/modules/ros2-nervous-system/nodes-topics.md",
        "docs/modules/ros2-nervous-system/urdf-modeling.md",
        "docs/modules/ros2-nervous-system/ai-integration.md",
        "docs/modules/ros2-nervous-system/references.md",
        "docs/modules/ros2-nervous-system/labs/lab1-workspace-setup.md",
        "docs/modules/digital-twin/gazebo-setup.md",
        "docs/modules/digital-twin/unity-integration.md",
        "docs/modules/digital-twin/sensor-simulation.md",
        "docs/modules/digital-twin/ros2-sync.md",
        "docs/modules/digital-twin/references.md",
        "docs/modules/digital-twin/labs/lab1-gazebo-setup.md"
    ]

    all_required = required_dirs + required_files + module_files

    missing_items = []

    print("\nChecking required directories...")
    for directory in required_dirs:
        if not check_directory_exists(directory):
            missing_items.append(f"DIR: {directory}")
        else:
            print(f"  ✓ {directory}")

    print("\nChecking required files...")
    for file in required_files + module_files:
        if not check_file_exists(file):
            missing_items.append(f"FILE: {file}")
        else:
            print(f"  ✓ {file}")

    # Check for potential Docusaurus configuration conflicts
    docusaurus_ok = check_for_nested_docusaurus_configs()

    print(f"\nValidation complete!")
    print(f"Found {len(all_required) - len(missing_items)} out of {len(all_required)} required items")

    if missing_items and not docusaurus_ok:
        print(f"\nMissing {len(missing_items)} items:")
        for item in missing_items:
            print(f"  - {item}")
        print("\nAdditionally, Docusaurus configuration conflicts were detected.")
        return False
    elif missing_items:
        print(f"\nMissing {len(missing_items)} items:")
        for item in missing_items:
            print(f"  - {item}")
        return False
    elif not docusaurus_ok:
        print("\nDocusaurus configuration conflicts detected.")
        return False
    else:
        print("\n🎉 All required items are present and Docusaurus configuration is clean!")
        return True


def main():
    """Main function to run the validation."""
    success = validate_project_structure()
    if not success:
        print("\n❌ Implementation is incomplete. Some required files/directories are missing or configuration conflicts exist.")
        sys.exit(1)
    else:
        print("\n✅ Implementation validation passed! All required components are present and configuration is clean.")
        sys.exit(0)


if __name__ == "__main__":
    main()