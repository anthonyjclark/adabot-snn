
# Building

```
cd sim
./build build <dir>
```

`<dir>` is a directory under `sim`. The main cpp file in `<dir>` must be named `<dir>.cpp`.

# Formatting

```
clang-format -i -style=Mozilla <file>
```

# Plotting

```
./bin/demo | gnuplot -p -e "set key outside; plot '<cat' using 1:3 with lines title columnheader"
```
