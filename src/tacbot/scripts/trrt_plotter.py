import pandas as pd
import os
import copy
import seaborn as sns
import matplotlib.pyplot as plt


class Reader:

    path = ""
    file = []
    df = pd.DataFrame()

    def __init__(self) -> None:
        pass

    def read(self, path: str, file: str) -> None:
        self.path = path
        self.file = file

        f_path = os.path.join(self.path, self.file)
        df = pd.read_csv(f_path)
        df.name = f_path
        self.df = df

    def get_df(self) -> pd.DataFrame:
        return copy.deepcopy(self.df)

    def set_df(self, df: pd.DataFrame) -> None:
        self.df = df


class Cleaner:
    df = pd.DataFrame()

    def ___init__(self) -> None:
        pass

    def set_df(self, df: pd.DataFrame) -> None:
        self.df = df

    def get_df(self) -> pd.DataFrame:
        return copy.deepcopy(self.df)

    def remove_outliers(self) -> None:
        ousted = self.df.index[(self.df["minDistToGoal"] > 100)]
        self.df.drop(ousted, inplace=True)
        ousted = self.df.index[(self.df["maxTemp"] < 0)]
        self.df.drop(ousted, inplace=True)
        print("{} outliers removed".format(
            len(ousted)))


class Plotter:

    fig_size = [20, 15]
    y_label = ""

    def __init__(self) -> None:
        self.init_figure()

    def init_figure(self) -> None:
        self.fig, self.ax = plt.subplots(
            figsize=(self.fig_size[0], self.fig_size[1]))

    def clear_ax(self) -> None:
        self.ax.cla()

    def plot(self, df: pd.DataFrame) -> None:
        sns.lineplot(x="sampleNumber", y="minDistToGoal",
                     data=df, color="b", lw=5, label="minDistToGoal")
        ax2 = plt.twinx()
        sns.lineplot(x="sampleNumber", y="temp",
                     data=df, ax=ax2, color="r", lw=5, label="temp")
        self.ax.legend(loc=9)
        ax2.legend(loc=0)
        plt.show()


sns.set(font_scale=2)

reader = Reader()
path = ""
file_name = "contactTRRT.csv"
reader.read(path, file_name)


cleaner = Cleaner()
cleaner.df = reader.df
cleaner.remove_outliers()

plotter = Plotter()
plotter.plot(cleaner.df)
