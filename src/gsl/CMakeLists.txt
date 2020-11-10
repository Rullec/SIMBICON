set(
GSL_SRC
error.c
block/block.c
block/file.c
block/init.c
vector/copy.c
vector/file.c
vector/init.c
vector/minmax.c
vector/oper.c
vector/prop.c
vector/swap.c
vector/vector.c
matrix/copy.c
matrix/file.c
matrix/getset.c
matrix/init.c
matrix/matrix.c
matrix/minmax.c
matrix/oper.c
matrix/prop.c
matrix/swap.c
blas/blas.c
blas/cblas.c

)



add_library(gsl ${GSL_SRC})
target_link_libraries(gsl)