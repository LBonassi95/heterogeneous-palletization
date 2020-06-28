import pandas as pd

def main():
    df = pd.read_csv('Test/results.csv', sep=",")
    print(df['TOT_BOXES'])


if __name__ == '__main__':
    main()









