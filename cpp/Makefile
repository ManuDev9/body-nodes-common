#
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

# Use > for indentation
.RECIPEPREFIX := >

buildir:
> mkdir -p build

BnRobotIK_ZYY2Arms.o:
> g++ -g -c BnRobotIK_ZYY2Arms.cpp -o build/BnRobotIK_ZYY2Arms.o

BnTwoNodesMotionTracking.o:
> g++ -g -c BnTwoNodesMotionTracking.cpp -o build/BnTwoNodesMotionTracking.o

BnReorientAxis.o:
> g++ -g -c BnReorientAxis.cpp -o build/BnReorientAxis.o

build: buildir BnReorientAxis.o BnTwoNodesMotionTracking.o BnRobotIK_ZYY2Arms.o
> g++ -g testBnCommon.cpp build/BnReorientAxis.o build/BnTwoNodesMotionTracking.o build/BnRobotIK_ZYY2Arms.o -o build/testBnCommon

test: build
> ./build/testBnCommon

clean:
> rm -r build

