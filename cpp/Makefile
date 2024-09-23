# Use > for indentation
.RECIPEPREFIX := >

BnRobotIK_ZYY2Arms.o:
> g++ -g -c BnRobotIK_ZYY2Arms.cpp -o BnRobotIK_ZYY2Arms.o

BnTwoNodesMotionTracking.o:
> g++ -g -c BnTwoNodesMotionTracking.cpp -o BnTwoNodesMotionTracking.o

BnReorientAxis.o:
> g++ -g -c BnReorientAxis.cpp -o BnReorientAxis.o

build: BnReorientAxis.o BnTwoNodesMotionTracking.o BnRobotIK_ZYY2Arms.o
> g++ -g testBnCommon.cpp BnReorientAxis.o BnTwoNodesMotionTracking.o BnRobotIK_ZYY2Arms.o -o testBnCommon

test: build
> ./testBnCommon

clean:
> rm testBnCommon *.o

