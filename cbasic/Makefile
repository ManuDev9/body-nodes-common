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

CFLAGS = -Wall -fPIC
LDFLAGS =

ifeq ($(COVERAGE),1)
CFLAGS += --coverage -O0 -g 
LDFLAGS += --coverage
endif

format:
> find src test/TestBnCommon.c -name '*.c' -o -name '*.h' | xargs clang-format -i

buildir: format
> mkdir -p build

BnAxisConfig.o: buildir src/BnAxisConfig.c src/BnAxisConfig.h
> gcc $(CFLAGS) -c src/BnAxisConfig.c -o build/BnAxisConfig.o

BnMotionTracking_2Nodes.o: buildir src/BnMotionTracking_2Nodes.c src/BnMotionTracking_2Nodes.h
> gcc $(CFLAGS) -c src/BnMotionTracking_2Nodes.c -o build/BnMotionTracking_2Nodes.o

BnQuaternion.o: buildir src/BnQuaternion.c src/BnQuaternion.h
> gcc $(CFLAGS) -c src/BnQuaternion.c -o build/BnQuaternion.o

BnRobotIK_ArmZYY.o: buildir src/BnRobotIK_ArmZYY.c src/BnRobotIK_ArmZYY.h
> gcc $(CFLAGS) -c src/BnRobotIK_ArmZYY.c -o build/BnRobotIK_ArmZYY.o

BnRobotArm_IKMT.o: buildir src/BnRobotArm_IKMT.c src/BnRobotArm_IKMT.h BnRobotIK_ArmZYY.o BnMotionTracking_2Nodes.o
> gcc $(CFLAGS) -c src/BnRobotArm_IKMT.c -Isrc -o build/BnRobotArm_IKMT.o

BnUtils.o: buildir src/BnUtils.c src/BnUtils.h
> gcc $(CFLAGS) -c src/BnUtils.c -Isrc -o build/BnUtils.o

library: buildir BnAxisConfig.o BnMotionTracking_2Nodes.o BnRobotIK_ArmZYY.o BnQuaternion.o BnRobotArm_IKMT.o BnUtils.o
> gcc $(CFLAGS) -shared -o build/libBnCommon.so \
    build/BnAxisConfig.o \
    build/BnMotionTracking_2Nodes.o \
    build/BnQuaternion.o \
    build/BnRobotIK_ArmZYY.o \
    build/BnRobotArm_IKMT.o \
    build/BnUtils.o \
    -lm

unity.o: buildir
> gcc $(CFLAGS) -c test/unity/unity.c -DUNITY_INCLUDE_DOUBLE -o build/unity.o

cJSON.o: buildir
> gcc $(CFLAGS) -c test/json/cJSON.c -o build/cJSON.o

build: buildir library unity.o cJSON.o
> gcc $(CFLAGS) -o build/TestBnCommon test/TestBnCommon.c -Isrc -Itest/unity -Itest/json build/unity.o build/cJSON.o -Lbuild -lBnCommon -lm $(LDFLAGS)

test: buildir build
> @LD_LIBRARY_PATH=build ./build/TestBnCommon

coverage: test
> make clean
> make COVERAGE=1 test
> gcovr -r . --object-directory build --sort-percent --exclude 'test/json/.*' --exclude 'test/unity/.*'

clean:
> rm -r build
