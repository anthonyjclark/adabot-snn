
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
./bin/demo | ./piplot.py --xname time
./piplot.py --file demo/data.csv --xname time
```

# Review Visualization

Use the following to add the `logger-cpp` submodule:

```bash
# Setup the submodule
git submodule add review/logger-cpp
git submodule update --init
cd logger-cpp
git remote rm origin
git remote add origin https://github.com/review/logger-cpp.git
```

```bash
# Push changes to logger-cpp
cd logger-cpp
git push origin master
```

```bash
# Pull changes
cd logger-cpp
git pull
```
