# Use > for indentation
.RECIPEPREFIX := >

BnReorientAxis.o:
> g++ -c BnReorientAxis.cpp -o BnReorientAxis.o

build: BnReorientAxis.o
> g++ testBnCommon.cpp BnReorientAxis.o -o testBnCommon

test: build
> ./testBnCommon

clean:
> rm testBnCommon *.o

