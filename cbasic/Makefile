# Use > for indentation
.RECIPEPREFIX := >

BnRobotIK_ZYY2Arms.o: BnRobotIK_ZYY2Arms.c BnRobotIK_ZYY2Arms.h
> gcc -c BnRobotIK_ZYY2Arms.c -o BnRobotIK_ZYY2Arms.o

BnTwoNodesMotionTracking.o: BnTwoNodesMotionTracking.c BnTwoNodesMotionTracking.h
> gcc -c BnTwoNodesMotionTracking.c -o BnTwoNodesMotionTracking.o

BnReorientAxis.o: BnReorientAxis.c BnReorientAxis.h
> gcc -c BnReorientAxis.c -o BnReorientAxis.o

build: BnReorientAxis.o BnTwoNodesMotionTracking.o BnRobotIK_ZYY2Arms.o
> g++ testBnCommon.cpp BnReorientAxis.o BnTwoNodesMotionTracking.o BnRobotIK_ZYY2Arms.o -o testBnCommon

test: build
> ./testBnCommon

clean:
> rm testBnCommon *.o

