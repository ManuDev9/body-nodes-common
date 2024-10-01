# Use > for indentation
.RECIPEPREFIX := >

buildir:
> mkdir -p build

BnRobotIK_ZYY2Arms.o: buildir BnRobotIK_ZYY2Arms.c BnRobotIK_ZYY2Arms.h
> gcc -c BnRobotIK_ZYY2Arms.c -o build/BnRobotIK_ZYY2Arms.o

BnTwoNodesMotionTracking.o: buildir BnTwoNodesMotionTracking.c BnTwoNodesMotionTracking.h
> gcc -c BnTwoNodesMotionTracking.c -o build/BnTwoNodesMotionTracking.o

BnReorientAxis.o: buildir BnReorientAxis.c BnReorientAxis.h
> gcc -c BnReorientAxis.c -o build/BnReorientAxis.o

build: buildir BnReorientAxis.o BnTwoNodesMotionTracking.o BnRobotIK_ZYY2Arms.o
> g++ testBnCommon.cpp build/BnReorientAxis.o build/BnTwoNodesMotionTracking.o build/BnRobotIK_ZYY2Arms.o -o build/testBnCommon

test: buildir build
> ./build/testBnCommon

clean:
> rm -r build

