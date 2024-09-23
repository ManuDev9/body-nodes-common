#!/bin/bash

output_txt=""

echo "Testing cbasic"
pushd cbasic
output_txt+=$(./run_tests.sh)$'\n'
popd

echo "Testing java"
pushd java
output_txt+=$(./run_tests.sh)$'\n'
popd

echo "Testing csharp"
pushd csharp
output_txt+=$(./run_tests.sh)$'\n'
popd

echo "Testing cpp"
pushd cpp
output_txt+=$(./run_tests.sh)$'\n'
popd

echo "Testing python"
pushd python
output_txt+=$(./run_tests.sh)$'\n'
popd
echo "$output_txt"


if echo "$output_txt" | grep -qi "fail"; then
    echo "Tests Failed!"
    exit 1  # Exit with error code
else
    echo "All tests passed!"
    exit 0  # Exit with success code
fi

