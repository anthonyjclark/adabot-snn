#!/usr/bin/env python3



# https://pandas.pydata.org/pandas-docs/stable/user_guide/visualization.html
# https://stackoverflow.com/questions/9622163/
# save-plot-to-image-file-instead-of-displaying-it-using-matplotlib

try:
    import pandas as pd
except ModuleNotFoundError as e:
    print("Could not import pandas")
    raise SystemExit

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError as e:
    print("Could not import matplotlib")
    raise SystemExit


from argparse import ArgumentParser
import sys


argparser = ArgumentParser(description="Plot the given file")
argparser.add_argument("-f", "--file", type=str, help="Plot data in a provided file")
argparser.add_argument("-x", "--xname", type=str, help="Name of column to use for x-axis")
argparser.add_argument("-y", "--yname", type=str, help="Name of column to use for y-axis")

args = argparser.parse_args()


if args.file:
    print(f"Reading data from: {args.file}")
    df = pd.read_csv(args.file, comment='#')
else:
    print(f"Reading data from stdin")
    df = pd.read_csv(sys.stdin, comment='#')


if df.empty:
    print("No data provided")
    raise SystemExit


if args.xname:
    df = df.set_index(args.xname)


with plt.xkcd():
    if args.yname:
        df[args.yname].plot(subplots=True, figsize=(12, 10))
    else:
        df.plot(subplots=True, figsize=(12, 10))
plt.show()
