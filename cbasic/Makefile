# Use > for indentation
.RECIPEPREFIX := >

BnReorientAxis.o:
> gcc -c BnReorientAxis.c -o BnReorientAxis.o

build: BnReorientAxis.o
> g++ testBnCommon.cpp BnReorientAxis.o -o testBnCommon

test: build
> ./testBnCommon

clean:
> rm testBnCommon *.o

