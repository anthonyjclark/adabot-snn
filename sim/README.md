
Change friction
see how it affects SNN vs diff drive model

Change frequency of data output

Inject noise into control signals


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

# Running

```
# Without visualization
./bin/fsm 5 5 12 1.1 0.4 0.8 2> fsm/fsm_data.csv

# With visualization
./bin/vis_fsm 5 5 12 1.1 0.4 0.8 1> fsm/fsm_vis.json 2> fsm/fsm_data.csv
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

# SNN

Inputs: control signals (left wheel, right wheel, and strut extension)

Outputs: delta position and angle



@book{tange_ole_2018_1146014,
      author       = {Tange, Ole},
      title        = {GNU Parallel 2018},
      publisher    = {Ole Tange},
      month        = Mar,
      year         = 2018,
      ISBN         = {9781387509881},
      doi          = {10.5281/zenodo.1146014},
      url          = {https://doi.org/10.5281/zenodo.1146014}
}
https://zenodo.org/record/1146014
