Small Instruction Set Virtual Machine is a simple, limited instruction set virtual machine.

### First time build

```aclocal
autoheader
autoconf
automake --add-missing
./configure
make
```

### Configure debug build

```mkdir debug
cd debug
../configure CFLAGS="-g -ggdb -O0 -Wall -fsanitize=address -ggdb3 -Wpedantic -fno-omit-frame-pointer -fprofile-arcs -ftest-coverage" && make
```
