import pandas as pd
from matplotlib import pyplot as plt


def main():
    df = pd.read_csv("./Test/results.csv")
    print df.keys()
    any = df[df["STRATEGY"] == "ANY"]
    id = df[df["STRATEGY"] == "ANY"]
    any.plot(x="TOT_BOXES", y="TIME_SOLUTION")
    plt.show()
    id.plot(x="TOT_BOXES", y="TIME_SOLUTION")
    plt.show()

if __name__ == '__main__':
    main()