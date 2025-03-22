#!/bin/bash
# MIT License
# 
# Copyright (c) 2024-2025 Manuel Bottini
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


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

