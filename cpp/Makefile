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
#

# Use > for indentation
.RECIPEPREFIX := >

buildir:
> mkdir -p build

BnAxisConfig.o: buildir src/BnAxisConfig.cpp src/BnAxisConfig.h
> g++ -fPIC -std=c++11 -c src/BnAxisConfig.cpp -o build/BnAxisConfig.o

BnMotionTracking_2Nodes.o: buildir src/BnMotionTracking_2Nodes.cpp src/BnMotionTracking_2Nodes.h
> g++ -fPIC -std=c++11 -c src/BnMotionTracking_2Nodes.cpp -o build/BnMotionTracking_2Nodes.o

BnQuaternion.o: buildir src/BnQuaternion.cpp src/BnQuaternion.h
> g++ -fPIC -std=c++11 -c src/BnQuaternion.cpp -o build/BnQuaternion.o

BnRobotIK_ArmZYY.o: buildir src/BnRobotIK_ArmZYY.cpp src/BnRobotIK_ArmZYY.h
> g++ -fPIC -std=c++11 -c src/BnRobotIK_ArmZYY.cpp -o build/BnRobotIK_ArmZYY.o

BnRobotArm_MT.o: buildir src/BnRobotArm_MT.cpp src/BnRobotArm_MT.h
> g++ -fPIC -std=c++11 -c src/BnRobotArm_MT.cpp -Isrc -o build/BnRobotArm_MT.o

BnUtils.o: buildir src/BnUtils.cpp src/BnUtils.h
> g++ -fPIC -std=c++11 -c src/BnUtils.cpp -Isrc -o build/BnUtils.o

library: buildir BnAxisConfig.o BnMotionTracking_2Nodes.o BnRobotIK_ArmZYY.o BnQuaternion.o BnRobotArm_MT.o BnUtils.o
> g++ -std=c++11 -shared -o build/libBnCommon.so \
    build/BnAxisConfig.o \
    build/BnMotionTracking_2Nodes.o \
    build/BnQuaternion.o \
    build/BnRobotIK_ArmZYY.o \
    build/BnRobotArm_MT.o \
    build/BnUtils.o \
    -lm

build: buildir library
> g++ -std=c++11 -o build/TestBnCommon test/TestBnCommon.cpp -Isrc -Itest -Lbuild -lBnCommon -lm

test: buildir build
> @LD_LIBRARY_PATH=build ./build/TestBnCommon -d yes

clean:
> rm -r build
