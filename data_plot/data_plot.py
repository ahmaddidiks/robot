#USAGE
# python plot_data.py --dataset <dataset> --xlabel <xlabel> --ylabel <ylabel>

#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-a", "--dataset", required=True, help="path to dataset")
ap.add_argument("-b", "--xlabel", required=True, help="xlabel")
ap.add_argument("-c", "--ylabel", required=True, help="ylabel")
args = vars(ap.parse_args())

class plot_data:
    def __init__(self, dataset):
        self.dataset = dataset

    def plot_data(self):
        df = pd.read_csv(self.dataset)
        key = list(df.keys())
        data_legend = []
        for i in key:
            plt.plot(df[i])
            data_legend.append(i)
        plt.xlabel(args["xlabel"])
        plt.ylabel(args["ylabel"])
        plt.title(args["dataset"].replace("_", " ").replace(".csv", "").replace("dataset/", " "))
        plt.legend(data_legend)
        plt.savefig(args["dataset"].replace(".csv", ".png").replace("dataset/", "output/"))
        plt.show()

if __name__ == "__main__":
    plot_data(args["dataset"]).plot_data()