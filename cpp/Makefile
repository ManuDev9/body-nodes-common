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

