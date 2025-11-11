#!/bin/bash
set -e

# Optional: build everything quietly
echo "🛠️  Building all packages..."
colcon build > /dev/null

echo -e "\n🧪 Running all GTest binaries in workspace:\n"

# Find and run all test executables in build/ directories
# (exclude non-executables and duplicates)
found=false
while IFS= read -r test_bin; do
    found=true
    echo "▶ $test_bin"
    "$test_bin" --gtest_color=yes
done < <(find build/ -type f -executable -name "test_*" | sort)

if ! $found; then
    echo "⚠️  No test executables found (none named test_*)"
fi

echo -e "\n✅ All GTests executed.\n"

# Optionally, also run colcon test for Python tests (pytest/unittest)
# Uncomment if you also have Python-based tests
# echo -e "🐍 Running Python tests via colcon...\n"
# colcon test --event-handlers console_direct+ && colcon test-result --verbose
