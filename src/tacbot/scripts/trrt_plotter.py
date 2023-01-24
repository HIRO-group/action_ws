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

    def line_plot(self, df: pd.DataFrame) -> None:
        sns.lineplot(x="sampleNumber", y="distToGoal", data=df, color="g")
        sns.lineplot(x="sampleNumber", y="minDistToGoal", data=df, color="b")
        ax2 = plt.twinx()
        sns.lineplot(x="sampleNumber", y="temp", data=df, ax=ax2, color="r")
        plt.show()


reader = Reader()
path = ""
file_name = "classicTRRT.csv"
reader.read(path, file_name)

plotter = Plotter()
plotter.line_plot(reader.df)
