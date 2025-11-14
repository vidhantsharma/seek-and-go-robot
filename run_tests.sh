#!/bin/bash
set -euo pipefail

echo "🛠️  Building all packages..."
# quiet build but still show colcon errors on failure
colcon build > /dev/null

echo -e "\n🧪 Running all GTest binaries in workspace:\n"

mkdir -p test_results
found=false

while IFS= read -r test_bin; do
    found=true
    name=$(basename "$test_bin")
    # produce both console output and xml file (XML at test_results/<binary>.xml)
    xmlfile="test_results/${name}.xml"
    echo "▶ $test_bin  (xml -> $xmlfile)"
    # run test and write xml (gtest) — some gtest variants accept --gtest_output
    "$test_bin" --gtest_color=yes --gtest_output=xml:"$xmlfile"
done < <(find build/ -type f -executable -name "test_*" | sort)

if ! $found; then
    echo "⚠️  No test executables found (none named test_*)"
fi

echo -e "\n✅ All GTests executed.\n"

# Optionally run colcon test for Python tests (uncomment if present)
# colcon test --event-handlers console_direct+ && colcon test-result --verbose
